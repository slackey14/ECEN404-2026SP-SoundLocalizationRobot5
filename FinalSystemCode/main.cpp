// =============================================================================
//  SOUND ROVER - FULLY INTEGRATED FIRMWARE
//  ESP32-S3 Wroom | ESP-IDF Framework | PlatformIO
// =============================================================================
//  Combines:
//    - 4-Mic Phase-Gradient TDOA Sound Localization
//    - Mecanum Wheel Navigation with A* Path Planning
//    - Obstacle Detection via 4x Ultrasonic Sensors
//    - WiFi Access Point + Web UI (map, status, manual control)
//
//  Architecture (FreeRTOS Tasks):
//    Core 0: WiFi/HTTP server  +  sound_task (I2S + DSP)
//    Core 1: nav_task (A* + motor control)
//
//  Shared state (protected by mutex):
//    g_sound_angle    - latest detected sound angle (degrees, 0=North/forward)
//    g_sound_distance - estimated distance to sound source (meters)
//    g_sound_valid    - true when a confident detection exists
//    g_sound_updated  - flag: nav_task consumes this to trigger navigation
//
//  Behaviour:
//    - Continuous sound listening; drives ONLY when "Chase Sound Source" is
//      clicked in the web UI (g_chase_requested flag).  The robot will NOT
//      autonomously start chasing sound on its own.
//    - If quiet: keeps listening, does NOT drive
//    - "Re-check every N steps" pauses mid-route, re-listens, re-plans if
//      heading changed > RECHECK_ANGLE_DELTA degrees
//    - Arrival detected by: magnitude >= ARRIVAL_MAG_FACTOR * start_magnitude
//      AND an obstacle within ARRIVAL_OBST_CM in the current travel direction.
//      This is far more reliable than inverse-square distance estimation.
//    - Manual start via web UI (direction + distance) still fully works
//    - Emergency stop from web UI halts both sound and nav
//
// =============================================================================

#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <math.h>  
#include <algorithm>  
#include <vector>  
#include <queue>  
#include <tuple>  
#include <complex>  
 
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "freertos/event_groups.h"  
#include "freertos/semphr.h"  
 
#include "esp_system.h"  
#include "esp_log.h"  
#include "esp_wifi.h"  
#include "esp_event.h"  
#include "esp_timer.h"  
#include "esp_rom_sys.h"  
#include "nvs_flash.h"  
 
#include "driver/gpio.h"  
#include "driver/ledc.h"  
#include "driver/i2s_std.h"  
 
#include "esp_http_server.h"  
 
using std::vector;  
using std::min;  
using std::max;  
using Complex = std::complex<float>;  
 
static const char* TAG = "SoundRover";  
 
// =============================================================================  
//  SOUND LOCALIZATION - CONFIGURATION  
// =============================================================================  
#define I2S_BCK_IO          GPIO_NUM_40  
#define I2S_WS_IO           GPIO_NUM_39  
#define I2S_DO_0_IO         GPIO_NUM_41   // I2S0 Data In (Mics 0 & 1)  
#define I2S_DO_1_IO         GPIO_NUM_42   // I2S1 Data In (Mics 2 & 3)  
 
#define SAMPLE_RATE         24000  
#define FFT_SIZE            1024  
#define BUFF_SIZE           FFT_SIZE  
#define SPEED_OF_SOUND      343.0f  
 
#define TARGET_FREQ         1000.0f       // Hz — beacon frequency to track
#define TARGET_SIGMA        5.0f          // Hz — Gaussian filter bandwidth
#define SMOOTHING           0.90f         // Temporal smoothing factor (0-1)
#define MAG_THRESHOLD       150000.0f     // Min FFT magnitude to consider valid
#define ANGLE_STEP          2             // Degrees per beamforming scan step

// Distance estimation via inverse-square law (used only as fallback for
// initial nav goal sizing; arrival is NOT determined by this value).
#define REFERENCE_ENERGY    0.05f
#define REFERENCE_DISTANCE  0.05f         // meters at reference energy
 
// ---------------------------------------------------------------------------  
//  ARRIVAL DETECTION  
//  
//  Arrival is declared purely by FFT magnitude growth — no ultrasonic check.  
//  This avoids false arrivals caused by ordinary walls or furniture, which  
//  would previously satisfy an "obstacle in front" condition.  
//  Normal obstacle avoidance (OBST_THRESH_CM) continues to work as usual for  
//  both manual and sound-chase navigation — arrival logic never interferes.  
//  
//  ARRIVAL_MAG_FACTOR: current magnitude must be >= start_mag * this value.  
//    Lower  = robot stops earlier (less magnitude growth required).  
//    Higher = robot must get much closer before stopping.  
//  
//  ARRIVAL_MIN_STEPS: robot must have taken at least this many steps before  
//    arrival can trigger.  Prevents a false stop on the very first frame  
//    if the room is reflective or the source is already loud.  
//  
//  SOUND_MIN_DISTANCE_IN: minimum initial goal distance in inches regardless  
//    of what the inverse-square energy estimator returns.  
// ---------------------------------------------------------------------------  
#define ARRIVAL_MAG_FACTOR      4.0f   // arrival mag must be >= N * start mag  
#define ARRIVAL_MIN_STEPS       3     // don't check arrival for first N steps  
#define SOUND_MIN_DISTANCE_IN   60.0f // minimum goal distance in inches
 
// Legacy — no longer used for arrival; kept so old references compile cleanly.  
#define SOUND_CLOSE_DIST    0.4f  
#define ARRIVAL_OBST_CM     50.0f  

// =============================================================================  
//  NAVIGATION RE-CHECK CONFIGURATION  
// =============================================================================  
#define RECHECK_INTERVAL    5       // steps between sound rechecks (0 = off)  
#define RECHECK_LISTEN_MS   500     // ms to listen during a recheck — 4-5 FFT
                                    // frames at 104ms each is enough for the
                                    // beamformer smoother (alpha=0.85) to settle
#define RECHECK_ANGLE_DELTA 15.0f   // degrees change needed to trigger re-plan  
 
// =============================================================================  
//  LED  
// =============================================================================  
#define RED_LED_PIN GPIO_NUM_38  
 
void setLED(bool on) { gpio_set_level(RED_LED_PIN, on ? 1 : 0); }  
 
void flashLED(int duration_ms = 120) {  
    gpio_set_level(RED_LED_PIN, 1);  
    vTaskDelay(pdMS_TO_TICKS(duration_ms));  
    gpio_set_level(RED_LED_PIN, 0);  
}  
 
void initLED() {  
    gpio_config_t cfg = {};  
    cfg.pin_bit_mask  = (1ULL << RED_LED_PIN);  
    cfg.mode          = GPIO_MODE_OUTPUT;  
    cfg.pull_up_en    = GPIO_PULLUP_DISABLE;  
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;  
    cfg.intr_type     = GPIO_INTR_DISABLE;  
    gpio_config(&cfg);  
    gpio_set_level(RED_LED_PIN, 0);  
}  
 
// =============================================================================  
//  ULTRASONIC SENSORS  
//  
//  OBST_THRESH_CM  — change this single value to adjust detection distance.  
//  15 inches = 38.1 cm  |  12 inches = 30.5 cm  |  24 inches = 61.0 cm  
//  
//  Uses GPIO edge interrupts + FreeRTOS semaphore instead of a busy-wait loop,  
//  so the CPU is properly yielded while waiting for the echo pulse.  
// =============================================================================  
#define OBST_THRESH_CM      25.4f    // cm — distance considered "blocked"  
#define US_SOUND_CM_PER_US  0.01715f // cm/us  (343 m/s / 2 for round-trip)  
#define US_TIMEOUT_US       25000    // 25 ms max wait -> ~4.3 m max range  
#define US_STABLE_READINGS  3        // readings averaged per measurement  
 
#define TRIG_LEFT   GPIO_NUM_1  
#define ECHO_LEFT   GPIO_NUM_2  
#define TRIG_BACKL   GPIO_NUM_4
#define ECHO_BACKL   GPIO_NUM_5  
#define TRIG_RIGHT  GPIO_NUM_10  
#define ECHO_RIGHT  GPIO_NUM_11  
#define TRIG_FRONTL  GPIO_NUM_36  
#define ECHO_FRONTL  GPIO_NUM_37  
#define TRIG_FRONTR GPIO_NUM_12
#define ECHO_FRONTR GPIO_NUM_35
#define TRIG_BACKR  GPIO_NUM_9
#define ECHO_BACKR  GPIO_NUM_16
 
// Per-sensor ISR state — each sensor has its own semaphore and rise/fall  
// timestamps so they can never corrupt each other's measurement.  
struct USSensor {  
    gpio_num_t        trig;  
    gpio_num_t        echo;  
    volatile int64_t  rise;  
    volatile int64_t  fall;  
    volatile bool     done;  
    SemaphoreHandle_t sem;  
};  
 
// Sensor table — index matches ECHO pin lookup in echo_isr via linear scan.  
// Declared before ISR so the ISR can find the right entry by echo pin number.  
static USSensor s_sensors[6]; // filled in initUltrasonicISR()  
 
static void IRAM_ATTR echo_isr(void* arg) {  
    gpio_num_t pin = (gpio_num_t)(intptr_t)arg;  
    // Find the matching sensor entry by echo pin  
    for (int i = 0; i < 6; i++) {  
        if (s_sensors[i].echo == pin) {  
            if (gpio_get_level(pin)) {  
                s_sensors[i].rise = esp_timer_get_time();  
            } else {  
                s_sensors[i].fall = esp_timer_get_time();  
                s_sensors[i].done = true;  
                BaseType_t woken = pdFALSE;  
                xSemaphoreGiveFromISR(s_sensors[i].sem, &woken);  
                portYIELD_FROM_ISR(woken);  
            }  
            return;  
        }  
    }  
}  
 
void initUltrasonicPins() {  
    gpio_num_t trigs[] = { TRIG_LEFT, TRIG_BACKL, TRIG_RIGHT, TRIG_FRONTL, TRIG_FRONTR, TRIG_BACKR };  
    for (gpio_num_t pin : trigs) {  
        gpio_config_t cfg = {};  
        cfg.pin_bit_mask = (1ULL << pin);  
        cfg.mode         = GPIO_MODE_OUTPUT;  
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;  
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;  
        cfg.intr_type    = GPIO_INTR_DISABLE;  
        gpio_config(&cfg);  
        gpio_set_level(pin, 0);  
    }  
    gpio_num_t echos[] = { ECHO_LEFT, ECHO_BACKL, ECHO_RIGHT, ECHO_FRONTL, ECHO_FRONTR, ECHO_BACKR };  
    for (gpio_num_t pin : echos) {  
        gpio_config_t cfg = {};  
        cfg.pin_bit_mask = (1ULL << pin);  
        cfg.mode         = GPIO_MODE_INPUT;  
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;  
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;  
        cfg.intr_type    = GPIO_INTR_DISABLE;  
        gpio_config(&cfg);  
    }  
}  
 
void initUltrasonicISR() {  
    // Build per-sensor table  
    gpio_num_t trigs[] = { TRIG_LEFT, TRIG_BACKL, TRIG_RIGHT, TRIG_FRONTL, TRIG_FRONTR, TRIG_BACKR };  
    gpio_num_t echos[] = { ECHO_LEFT, ECHO_BACKL, ECHO_RIGHT, ECHO_FRONTL, ECHO_FRONTR, ECHO_BACKR };  
    for (int i = 0; i < 6; i++) {  
        s_sensors[i].trig = trigs[i];  
        s_sensors[i].echo = echos[i];  
        s_sensors[i].rise = 0;  
        s_sensors[i].fall = 0;  
        s_sensors[i].done = false;  
        s_sensors[i].sem  = xSemaphoreCreateBinary();  
    }  
 
    ESP_ERROR_CHECK(gpio_install_isr_service(0));  
    for (int i = 0; i < 6; i++) {  
        ESP_ERROR_CHECK(gpio_set_intr_type(echos[i], GPIO_INTR_ANYEDGE));  
        ESP_ERROR_CHECK(gpio_isr_handler_add(echos[i], echo_isr, (void*)(intptr_t)echos[i]));  
    }  
    ESP_LOGI(TAG, "Ultrasonic ISR installed (per-sensor, 6 sensors) — threshold %.1f cm (%.1f in)",  
             OBST_THRESH_CM, OBST_THRESH_CM / 2.54f);  
}  
 
// Returns the USSensor entry for a given echo pin, or nullptr if not found.  
static USSensor* sensorByEcho(gpio_num_t echopin) {  
    for (int i = 0; i < 6; i++)  
        if (s_sensors[i].echo == echopin) return &s_sensors[i];  
    return nullptr;  
}  
 
static float distanceCm(gpio_num_t trigpin, gpio_num_t echopin) {  
    USSensor* s = sensorByEcho(echopin);  
    if (!s) return 999.0f;  
 
    // Drain any stale token from a previous trigger  
    xSemaphoreTake(s->sem, 0);  
    s->done = false;  
    s->rise = 0;  
    s->fall = 0;  
 
    gpio_set_level(trigpin, 0); esp_rom_delay_us(2);  
    gpio_set_level(trigpin, 1); esp_rom_delay_us(10);  
    gpio_set_level(trigpin, 0);  
 
    if (xSemaphoreTake(s->sem, pdMS_TO_TICKS(35)) != pdTRUE) {  
        return 999.0f;  
    }  
    if (!s->done || s->fall <= s->rise) return 999.0f;  
 
    int64_t duration_us = s->fall - s->rise;  
    if (duration_us > US_TIMEOUT_US) return 999.0f;  
 
    return (float)duration_us * US_SOUND_CM_PER_US;  
}  
 
float getStableDistance(gpio_num_t trigpin, gpio_num_t echopin) {  
    float sum   = 0.0f;  
    int   valid = 0;  
    for (int i = 0; i < US_STABLE_READINGS; i++) {  
        float d = distanceCm(trigpin, echopin);  
        if (d < 999.0f) { sum += d; valid++; }  
        vTaskDelay(pdMS_TO_TICKS(10));  
    }  
    if (valid == 0) return 999.0f;  
    float avg = sum / (float)valid;  
    return (avg < 2.0f) ? 999.0f : avg;  
}  
 
// =============================================================================  
//  MOTOR PINS & LEDC  
// =============================================================================  
#define MF_PWMA  GPIO_NUM_45  
#define MF_AI1   GPIO_NUM_47  
#define MF_AI2   GPIO_NUM_48  
#define MF_PWMB  GPIO_NUM_13  
#define MF_BI1   GPIO_NUM_21  
#define MF_BI2   GPIO_NUM_14  
#define MR_PWMA  GPIO_NUM_8  
#define MR_AI1   GPIO_NUM_6  
#define MR_AI2   GPIO_NUM_7  
#define MR_PWMB  GPIO_NUM_18  
#define MR_BI1   GPIO_NUM_15  
#define MR_BI2   GPIO_NUM_17  
 
#define LEDC_FREQ_HZ    1000  //pwm
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT  
 
#define LEDC_TIMER_RF   LEDC_TIMER_0  
#define LEDC_TIMER_LF   LEDC_TIMER_1  
#define LEDC_TIMER_RR   LEDC_TIMER_2  
#define LEDC_TIMER_LR   LEDC_TIMER_3  
#define LEDC_CH_RF      LEDC_CHANNEL_0  
#define LEDC_CH_LF      LEDC_CHANNEL_1  
#define LEDC_CH_RR      LEDC_CHANNEL_2  
#define LEDC_CH_LR      LEDC_CHANNEL_3  
 
void initMotorPWM() {  
    ledc_timer_config_t tc = {};  
    tc.speed_mode      = LEDC_LOW_SPEED_MODE;  
    tc.duty_resolution = LEDC_RESOLUTION;  
    tc.freq_hz         = LEDC_FREQ_HZ;  
    tc.clk_cfg         = LEDC_AUTO_CLK;  
    tc.timer_num = LEDC_TIMER_RF; ledc_timer_config(&tc);  
    tc.timer_num = LEDC_TIMER_LF; ledc_timer_config(&tc);  
    tc.timer_num = LEDC_TIMER_RR; ledc_timer_config(&tc);  
    tc.timer_num = LEDC_TIMER_LR; ledc_timer_config(&tc);  
 
    ledc_channel_config_t ch = {};  
    ch.speed_mode = LEDC_LOW_SPEED_MODE;  
    ch.intr_type  = LEDC_INTR_DISABLE;  
    ch.duty = 0; ch.hpoint = 0;  
    ch.channel = LEDC_CH_RF; ch.gpio_num = MF_PWMA; ch.timer_sel = LEDC_TIMER_RF; ledc_channel_config(&ch);  
    ch.channel = LEDC_CH_LF; ch.gpio_num = MF_PWMB; ch.timer_sel = LEDC_TIMER_LF; ledc_channel_config(&ch);  
    ch.channel = LEDC_CH_RR; ch.gpio_num = MR_PWMA; ch.timer_sel = LEDC_TIMER_RR; ledc_channel_config(&ch);  
    ch.channel = LEDC_CH_LR; ch.gpio_num = MR_PWMB; ch.timer_sel = LEDC_TIMER_LR; ledc_channel_config(&ch);  
}  
 
void setMotorPWM(ledc_channel_t ch, int duty) {  
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, (uint32_t)abs(duty));  
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);  
}  
 
void initMotorGPIO() {  
    gpio_num_t pins[] = { MF_AI1, MF_AI2, MF_BI1, MF_BI2,  
                          MR_AI1, MR_AI2, MR_BI1, MR_BI2 };  
    for (gpio_num_t pin : pins) {  
        gpio_config_t cfg = {};  
        cfg.pin_bit_mask = (1ULL << pin);  
        cfg.mode         = GPIO_MODE_OUTPUT;  
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;  
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;  
        cfg.intr_type    = GPIO_INTR_DISABLE;  
        gpio_config(&cfg);  
        gpio_set_level(pin, 0);  
    }  
}  
 
// =============================================================================  
//  MECANUM DIRECTION BYTES  
// =============================================================================  
const uint8_t MEC_STRAIGHT_FORWARD  = 0b01100110;  
const uint8_t MEC_STRAIGHT_BACKWARD = 0b10011001;  
const uint8_t MEC_SIDEWAYS_LEFT     = 0b01011010; 
const uint8_t MEC_SIDEWAYS_RIGHT    = 0b10100101;  
 
// =============================================================================  
//  NAVIGATION GLOBALS  
// =============================================================================  
const int MAX_GRID  = 60;  
const int EXPAND_BY = 10;  
const int MAX_STEPS = 300;  
 
vector<vector<int>> grid;  
vector<std::pair<int,int>> visited;  
int curX = 0, curY = 0, goalX = 0, goalY = 0;
int origStartX = -1, origStartY = -1;
int  currentStep       = 0;  
bool navigationActive  = false;  
bool emergencyStop     = false;  
bool soundNavMode      = false;  
// FIX 1: g_orig_start_set ensures origStartX/Y is only written once per
// fresh navigation session, never overwritten by mid-run replans or rechecks.
bool g_orig_start_set  = false;
char statusMessage[256] = "Idle — press Chase Sound Source to begin.";  
 
// Timer state
static int64_t g_nav_start_time = 0;
static int64_t g_nav_elapsed_us = 0;

// FIX 2: Total grid steps traveled; each step = 9 in.
// Reset only on a genuine fresh start, not on replans.
static int g_total_steps_traveled = 0;

// Displacement Tracking (Immune to grid shifts)
// Represents net steps taken since the true start.
static int g_net_dx = 0;
static int g_net_dy = 0;

// Strafe PWM values configured via web UI (defaults requested by user)
int g_pwm_left_rf=229,  g_pwm_left_lf=235,  g_pwm_left_rr=237,  g_pwm_left_lr=230;
int g_pwm_right_rf=230, g_pwm_right_lf=240, g_pwm_right_rr=240, g_pwm_right_lr=230;

// Current travel direction (updated each step; used for arrival obstacle check)  
static char g_current_dir[16] = "forward";  
 
// =============================================================================  
//  SHARED SOUND STATE  (mutex-protected, written by sound_task)  
// =============================================================================  
static SemaphoreHandle_t g_sound_mutex = NULL;  
static float g_sound_angle    = 0.0f;  
static float g_sound_distance = -1.0f;  
static bool  g_sound_valid    = false;  
static bool  g_sound_updated  = false;  
static float g_sound_rms[4]   = {};  
static float g_sound_mag      = 0.0f;   // latest average magnitude at peak  
 
// Magnitude snapshot taken when Chase Sound Source is clicked.  
// Arrival check compares live magnitude against this baseline.  
static float g_chase_start_mag = 0.0f;  
 
// Set by handler_sound_start (web button); consumed by nav_task.  
// nav_task NEVER self-starts — it waits for this flag.  
static bool  g_chase_requested = false;  
 
// I2S handles  
static i2s_chan_handle_t rx_handle_0 = NULL;  
static i2s_chan_handle_t rx_handle_1 = NULL;  
 
// =============================================================================  
//  DSP GLOBALS  
// =============================================================================  
static const float PI_F = 3.14159265358979323846f;
static int g_recheck_interval = RECHECK_INTERVAL;

// Runtime-configurable target frequency (default = compile-time TARGET_FREQ).
// Changing this via /set_freq rebuilds DSP tables and restarts the scan band.
static float g_target_freq = TARGET_FREQ;

// Runtime-configurable move durations (ms).
// Forward/backward and side-to-side can use different values.
#define MOVE_DELAY_FWD_MS   235   // default for forward / backward
#define MOVE_DELAY_SIDE_MS  235   // default for left / right (strafe)
static int g_move_delay_fwd_ms  = MOVE_DELAY_FWD_MS;
static int g_move_delay_side_ms = MOVE_DELAY_SIDE_MS;

// Absolute physical start — recorded ONCE per fresh session (never shifted).
// Unlike origStartX/Y (which live in grid coordinates and move when the grid
// expands/shifts), these store the robot's real-world step-count offsets from
// the session origin so the return path targets the exact launch point.
// We track it as cumulative (dx, dy) steps from the true first position.
static int g_abs_start_grid_x = -1;
static int g_abs_start_grid_y = -1;
static bool g_abs_start_valid = false;

// =============================================================================
//  MIC GEOMETRY  (SRP-PHAT beamformer)
//  0,0 is center of the array. +Y = forward (front of rover), +X = right.
//  0 deg = forward (+Y), 90 deg = right (+X)  — navigation coordinates.
// =============================================================================
struct Point { float x, y; };

const Point MIC_POSITIONS[4] = {
    { 0.03175f, 0.03175f },   // Mic 0
    { 0.03175f,  -0.03175f },   // Mic 1
    { -0.03175f, 0.03175f },  // Mic 2
    { -0.03175f,  -0.03175f },  // Mic 3
};

// Precomputed SRP-PHAT steering vectors and gaussian frequency weights
static vector<vector<Complex>> s_steering_vectors;
static vector<Complex>         s_smoothed_bins(4, Complex(0.0f, 0.0f));
static vector<float>           s_gaussian_weights;
static float                   g_last_valid_angle = 0.0f;
 
// =============================================================================  
//  MATH / DSP HELPERS  
// =============================================================================  
void apply_hann_window(vector<float>& data) {  
    for (size_t i = 0; i < data.size(); ++i) {  
        float m = 0.5f * (1.0f - cosf(2.0f * PI_F * i / (data.size() - 1)));  
        data[i] *= m;  
    }  
}  
 
float calculate_rms_energy(const vector<float>& frame) {  
    if (frame.empty()) return 0.0f;  
    float sum_sq = 0.0f;  
    for (float s : frame) { float n = s / 8388608.0f; sum_sq += n * n; }  
    return sqrtf(sum_sq / frame.size());  
}  
 
// Iterative Cooley-Tukey FFT (radix-2, DIT, in-place) - highly stable
void fft(vector<Complex>& a) {
    int n = a.size();
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * PI_F / (float)len;
        Complex wlen(cosf(ang), sinf(ang));
        for (int i = 0; i < n; i += len) {
            Complex w(1.0f, 0.0f);
            for (int j = 0; j < len / 2; j++) {
                Complex u = a[i+j];
                Complex v = a[i+j+len/2] * w;
                a[i+j]       = u + v;
                a[i+j+len/2] = u - v;
                w *= wlen;
            }
        }
    }
}
 
// =============================================================================
//  I2S INIT
// =============================================================================

// --- PRECOMPUTE DSP (SRP-PHAT steering vectors + Gaussian weights) ---
// Must be called once before sound_task starts.
// Uses navigation coordinates: 0 deg = forward (+Y), 90 deg = right (+X).
void precompute_dsp() {
    int num_angles = 360 / ANGLE_STEP;
    s_steering_vectors.resize(num_angles);
    float omega = 2.0f * PI_F * g_target_freq;

    for (int i = 0; i < num_angles; ++i) {
        int   angle     = i * ANGLE_STEP;
        float angle_rad = angle * PI_F / 180.0f;
        // Navigation coords: x = sin(theta), y = cos(theta)
        float k_x = sinf(angle_rad);
        float k_y = cosf(angle_rad);

        s_steering_vectors[i].resize(4);
        for (int mic = 0; mic < 4; ++mic) {
            float dist_proj = MIC_POSITIONS[mic].x * k_x + MIC_POSITIONS[mic].y * k_y;
            float delay     = dist_proj / SPEED_OF_SOUND;
            // Negative phase aligns the incoming wavefront to the array center
            s_steering_vectors[i][mic] = std::polar(1.0f, -omega * delay);
        }
    }

    // Gaussian weights centred on g_target_freq with sigma = TARGET_SIGMA
    s_gaussian_weights.resize(FFT_SIZE / 2, 0.0f);
    for (int k = 0; k < FFT_SIZE / 2; k++) {
        float freq = (float)k * SAMPLE_RATE / FFT_SIZE;
        float diff = freq - g_target_freq;
        s_gaussian_weights[k] = expf(-(diff * diff) / (2.0f * TARGET_SIGMA * TARGET_SIGMA));
    }

    // Scan band: +/- 3 sigma covers 99 % of the Gaussian bell
    float scan_bw = TARGET_SIGMA * 3.0f;
    int sb = (int)((g_target_freq - scan_bw) * FFT_SIZE / SAMPLE_RATE);
    int eb = (int)((g_target_freq + scan_bw) * FFT_SIZE / SAMPLE_RATE);
    if (sb < 1)          sb = 1;
    if (eb >= FFT_SIZE/2) eb = FFT_SIZE/2 - 1;

    ESP_LOGI(TAG, "[DSP] Precomputed. Target: %.0f Hz, Sigma: %.0f Hz, Scan bins %d-%d",
             g_target_freq, TARGET_SIGMA, sb, eb);
}

void init_microphone_i2s() {  
    // I2S0 MASTER
    i2s_chan_config_t cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);  
    cfg0.auto_clear = true;  
    cfg0.dma_desc_num = 8;
    cfg0.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&cfg0, NULL, &rx_handle_0));  
   
    i2s_std_config_t std0 = {  
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),  
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),  
        .gpio_cfg = {  
            .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCK_IO, .ws = I2S_WS_IO,  
            .dout = I2S_GPIO_UNUSED, .din  = I2S_DO_0_IO,  
            .invert_flags = { false, false, false },  
        },  
    };  
    std0.clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M; // Highly accurate internal clock
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_0, &std0));  
 
    // I2S1 SLAVE
    i2s_chan_config_t cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);  
    cfg1.auto_clear = true;  
    cfg1.dma_desc_num = 8;
    cfg1.dma_frame_num = 512;
    ESP_ERROR_CHECK(i2s_new_channel(&cfg1, NULL, &rx_handle_1));  
   
    i2s_std_config_t std1 = {  
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),  
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),  
        .gpio_cfg = {  
            .mclk = I2S_GPIO_UNUSED, .bclk = I2S_BCK_IO, .ws = I2S_WS_IO,  
            .dout = I2S_GPIO_UNUSED, .din  = I2S_DO_1_IO,  
            .invert_flags = { false, false, false },  
        },  
    };  
    std1.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT; // Explicitly clear for slave
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_1, &std1));  
 
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_0));  
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_1));  
    ESP_LOGI(TAG, "I2S microphones initialized (Master/Slave locked) @ %d Hz", SAMPLE_RATE);  
}  
 
// =============================================================================
//  SOUND LOCALIZATION TASK  (Core 0, alongside WiFi)
//  SRP-PHAT beamformer with precomputed steering vectors.
//  0 deg = forward (+Y). Positive angle = clockwise (navigation convention).
// =============================================================================

// Parabolic interpolation for sub-degree angle precision
static float interpolate_peak(float y_minus, float y_center, float y_plus, int center_angle) {
    float denom = y_minus - 2.0f * y_center + y_plus;
    if (denom == 0.0f) return (float)center_angle;
    float delta = (y_minus - y_plus) / (2.0f * denom);
    return (float)center_angle + (delta * ANGLE_STEP);
}

static float estimate_distance_srp(float measured_energy) {
    if (measured_energy < 1e-9f) return -1.0f;
    return REFERENCE_DISTANCE * sqrtf(REFERENCE_ENERGY / measured_energy);
}

void sound_task(void* param) {
    // Gaussian scan band: 3-sigma covers 99% of the filter bell
    float scan_bw  = TARGET_SIGMA * 3.0f;
    int start_bin  = (int)((g_target_freq - scan_bw) * FFT_SIZE / SAMPLE_RATE);
    int end_bin    = (int)((g_target_freq + scan_bw) * FFT_SIZE / SAMPLE_RATE);
    if (start_bin < 1)           start_bin = 1;
    if (end_bin >= FFT_SIZE / 2) end_bin   = FFT_SIZE / 2 - 1;

    float last_known_freq = g_target_freq;  // detect freq changes mid-task

    ESP_LOGI(TAG, "[Sound] SRP-PHAT | Target %.0f Hz | Bins %d-%d | Sigma %.0f Hz",
             g_target_freq, start_bin, end_bin, TARGET_SIGMA);

    int32_t* raw0 = (int32_t*)calloc(BUFF_SIZE * 2, sizeof(int32_t));
    int32_t* raw1 = (int32_t*)calloc(BUFF_SIZE * 2, sizeof(int32_t));

    vector<vector<float>>   td(4, vector<float>(FFT_SIZE));
    vector<vector<Complex>> fd(4, vector<Complex>(FFT_SIZE));

    int num_angles          = 360 / ANGLE_STEP;
    int quiet_print_counter = 0;

    while (true) {
        // If g_target_freq was changed via /set_freq, recompute scan band + DSP
        if (g_target_freq != last_known_freq) {
            last_known_freq = g_target_freq;
            precompute_dsp();
            scan_bw   = TARGET_SIGMA * 3.0f;
            start_bin = (int)((g_target_freq - scan_bw) * FFT_SIZE / SAMPLE_RATE);
            end_bin   = (int)((g_target_freq + scan_bw) * FFT_SIZE / SAMPLE_RATE);
            if (start_bin < 1)           start_bin = 1;
            if (end_bin >= FFT_SIZE / 2) end_bin   = FFT_SIZE / 2 - 1;
            // Reset smoother so stale phasors from the old frequency don't bleed in
            for (int m = 0; m < 4; m++) s_smoothed_bins[m] = Complex(0.0f, 0.0f);
            ESP_LOGI(TAG, "[Sound] Freq changed -> %.0f Hz, bins %d-%d",
                     g_target_freq, start_bin, end_bin);
        }

        size_t bytesA = 0, bytesB = 0;
        const size_t readBytes = (size_t)(BUFF_SIZE * 2) * sizeof(int32_t);

        // Read SLAVE first, then MASTER (matches micsonly.cpp timing)
        esp_err_t r1 = i2s_channel_read(rx_handle_1, raw1, readBytes, &bytesB, 100);
        esp_err_t r0 = i2s_channel_read(rx_handle_0, raw0, readBytes, &bytesA, 100);

        if (r0 != ESP_OK || r1 != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Deinterleave stereo buffers into 4 mic channels
        for (int i = 0; i < FFT_SIZE; i++) {
            td[0][i] = (float)(raw0[i*2]   >> 8);  // Mic 0
            td[1][i] = (float)(raw0[i*2+1] >> 8);  // Mic 1
            td[2][i] = (float)(raw1[i*2]   >> 8);  // Mic 2
            td[3][i] = (float)(raw1[i*2+1] >> 8);  // Mic 3
        }

        // RMS energy per mic (pre-window, for distance estimation)
        float rms[4];
        for (int m = 0; m < 4; m++) rms[m] = calculate_rms_energy(td[m]);

        // Hann window + FFT
        for (int m = 0; m < 4; m++) {
            apply_hann_window(td[m]);
            for (int k = 0; k < FFT_SIZE; k++) fd[m][k] = Complex(td[m][k], 0.0f);
            fft(fd[m]);
        }

        // --- SRP-PHAT: Gaussian-weighted phasor extraction ---
        float   max_mag = 0.0f;
        Complex weighted_phasors[4] = {};

        for (int k = start_bin; k <= end_bin; k++) {
            float weight = s_gaussian_weights[k];
            if (weight < 0.01f) continue;

            for (int m = 0; m < 4; m++) {
                float mag = std::abs(fd[m][k]);
                if (mag > max_mag) max_mag = mag;

                // Unit-normalise each bin (PHAT), then apply Gaussian weight
                if (mag > 0.0001f) {
                    Complex phat = fd[m][k] / mag;
                    weighted_phasors[m] += phat * weight;
                }
            }
        }

        float final_angle  = g_last_valid_angle;
        float display_dist = -1.0f;
        float final_mag    = 0.0f;

        if (max_mag > MAG_THRESHOLD) {
            quiet_print_counter = 0;

            // Temporal smoothing of weighted phasors
            for (int m = 0; m < 4; m++) {
                s_smoothed_bins[m] = (SMOOTHING * s_smoothed_bins[m])
                                   + ((1.0f - SMOOTHING) * weighted_phasors[m]);
            }

            // Full angular beamforming scan
            float best_energy = -1.0f;
            int   best_idx    = 0;
            vector<float> energies(num_angles);

            for (int i = 0; i < num_angles; ++i) {
                Complex beam_sum(0.0f, 0.0f);
                for (int m = 0; m < 4; m++) {
                    beam_sum += s_smoothed_bins[m] * s_steering_vectors[i][m];
                }
                float e = std::norm(beam_sum);
                energies[i] = e;
                if (e > best_energy) { best_energy = e; best_idx = i; }
            }

            // Parabolic interpolation for sub-degree precision
            int prev_idx = (best_idx == 0)              ? num_angles - 1 : best_idx - 1;
            int next_idx = (best_idx == num_angles - 1) ? 0              : best_idx + 1;
            final_angle  = interpolate_peak(energies[prev_idx], energies[best_idx],
                                            energies[next_idx], best_idx * ANGLE_STEP);

            // Wrap to [0, 360)
            if (final_angle <    0.0f) final_angle += 360.0f;
            if (final_angle >= 360.0f) final_angle -= 360.0f;

            g_last_valid_angle = final_angle;

            display_dist = estimate_distance_srp(rms[0]);
            final_mag    = max_mag;

            if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_sound_angle    = final_angle;
                g_sound_distance = display_dist;
                g_sound_valid    = true;
                g_sound_updated  = true;
                g_sound_mag      = final_mag;
                for (int m = 0; m < 4; m++) g_sound_rms[m] = rms[m];
                xSemaphoreGive(g_sound_mutex);
            }

            printf("[Sound] M0:%.4f M1:%.4f M2:%.4f M3:%.4f | Angle=%5.1f deg | Dist=%.3fm | Mag=%.0f\n",
                   rms[0], rms[1], rms[2], rms[3], final_angle, display_dist, max_mag);

        } else {
            // Quiet: gently decay smoothed phasors
            for (int m = 0; m < 4; m++) s_smoothed_bins[m] *= 0.9f;

            if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_sound_valid = false;
                g_sound_mag   = 0.0f;
                xSemaphoreGive(g_sound_mutex);
            }

            if (++quiet_print_counter >= 25) {
                quiet_print_counter = 0;
                printf("[Sound] M0:%.4f M1:%.4f M2:%.4f M3:%.4f | (Quiet - listening for %.0fHz...)\n",
                       rms[0], rms[1], rms[2], rms[3], g_target_freq);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }

    free(raw0);
    free(raw1);
}

 
// =============================================================================  
//  SOUND RECHECK — blocks for RECHECK_LISTEN_MS and returns best detection  
// =============================================================================  
bool recheckSound(float& out_angle, float& out_dist) {  
    printf("\n[Recheck] Listening %d ms for %.0fHz...\n", RECHECK_LISTEN_MS, g_target_freq);  
    snprintf(statusMessage, sizeof(statusMessage), "Rechecking sound...");  
 
    int64_t deadline  = esp_timer_get_time() + (int64_t)RECHECK_LISTEN_MS * 1000;  
    bool    found     = false;  
    float   best_ang  = 0.0f, best_dist = -1.0f, best_mag = 0.0f;
 
    while (esp_timer_get_time() < deadline) {  
        if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {  
            if (g_sound_valid && g_sound_mag > best_mag) {
                best_ang  = g_sound_angle;  
                best_dist = g_sound_distance;
                best_mag  = g_sound_mag;
                found     = true;  
            }  
            xSemaphoreGive(g_sound_mutex);  
        }  
        vTaskDelay(pdMS_TO_TICKS(50));  
    }  
 
    if (found)  
        printf("[Recheck] Detected: angle=%.1f deg dist=%.3fm mag=%.0f\n", best_ang, best_dist, best_mag);  
    else  
        printf("[Recheck] No sound detected — keeping current goal\n");  
 
    out_angle = best_ang;  
    out_dist  = best_dist;  
    return found;  
}  
 
// =============================================================================  
//  ARRIVAL CHECK  (sound nav only)  
//  
//  Returns true when:  
//    - At least ARRIVAL_MIN_STEPS have been taken (avoids false trigger at start)  
//    - Current FFT magnitude >= g_chase_start_mag * ARRIVAL_MAG_FACTOR  
//  
//  Normal obstacle avoidance (OBST_THRESH_CM) is completely unaffected and  
//  continues to work for both manual and sound-chase navigation.  
//  This function never touches the ultrasonic sensors.  
// =============================================================================  
// Number of consecutive checkArrival() calls that must return true before
// arrival is declared.  Each call is separated by one nav loop iteration
// (~440 ms of motor + delay time), so 2 = ~880 ms of sustained magnitude.
#define ARRIVAL_CONFIRM_FRAMES  2

// Persistent counter — reset whenever the check fails or nav restarts.
static int s_arrival_confirm = 0;

bool checkArrival() {  
    if (!soundNavMode) return false;  
    if (g_chase_start_mag <= 0.0f) return false;  
    if (currentStep < ARRIVAL_MIN_STEPS) { s_arrival_confirm = 0; return false; }  
 
    float live_mag = 0.0f;  
    if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {  
        live_mag = g_sound_mag;  
        xSemaphoreGive(g_sound_mutex);  
    }  
 
    float threshold_mag = g_chase_start_mag * ARRIVAL_MAG_FACTOR;  
    bool  above         = (live_mag >= threshold_mag);

    if (above) {
        s_arrival_confirm++;
    } else {
        s_arrival_confirm = 0;
    }
    bool arrived = (s_arrival_confirm >= ARRIVAL_CONFIRM_FRAMES);
 
    printf("[Arrival] step=%d  live_mag=%.0f  start_mag=%.0f  threshold=%.0f  confirm=%d/%d  arrived=%d\n",  
           currentStep, live_mag, g_chase_start_mag, threshold_mag,
           s_arrival_confirm, ARRIVAL_CONFIRM_FRAMES, (int)arrived);  
 
    return arrived;  
}  
 
// =============================================================================  
//  MOTOR FUNCTIONS  
// =============================================================================  
void moveMotors(int speedRF, int speedLF, int speedRR, int speedLR, uint8_t dir) {  
    if (emergencyStop) return;  
    gpio_set_level(MF_AI1, (dir>>7)&1); gpio_set_level(MF_AI2, (dir>>6)&1);  
    gpio_set_level(MF_BI1, (dir>>5)&1); gpio_set_level(MF_BI2, (dir>>4)&1);  
    gpio_set_level(MR_AI1, (dir>>3)&1); gpio_set_level(MR_AI2, (dir>>2)&1);  
    gpio_set_level(MR_BI1, (dir>>1)&1); gpio_set_level(MR_BI2, (dir>>0)&1);  
    setMotorPWM(LEDC_CH_RF, abs(speedRF));  
    setMotorPWM(LEDC_CH_LF, abs(speedLF));  
    setMotorPWM(LEDC_CH_RR, abs(speedRR));  
    setMotorPWM(LEDC_CH_LR, abs(speedLR));  
    flashLED(100);  
}  
 
void stopMotors() {  
    setMotorPWM(LEDC_CH_RF,0); setMotorPWM(LEDC_CH_LF,0);  
    setMotorPWM(LEDC_CH_RR,0); setMotorPWM(LEDC_CH_LR,0);  
    gpio_set_level(MF_AI1,0); gpio_set_level(MF_AI2,0);  
    gpio_set_level(MF_BI1,0); gpio_set_level(MF_BI2,0);  
    gpio_set_level(MR_AI1,0); gpio_set_level(MR_AI2,0);  
    gpio_set_level(MR_BI1,0); gpio_set_level(MR_BI2,0);  
    flashLED(200);  
}  
 
void driveMecanum(const char* direction) {  
    if (emergencyStop) return;  
    if      (strcmp(direction, "forward") == 0) moveMotors(230, 230, 230, 230, MEC_STRAIGHT_FORWARD);  //speed of motors
    else if (strcmp(direction, "backward") == 0) moveMotors(230, 230, 230, 230, MEC_STRAIGHT_BACKWARD);  
    else if (strcmp(direction, "right") == 0) moveMotors(g_pwm_right_rf, g_pwm_right_lf, g_pwm_right_rr, g_pwm_right_lr, MEC_SIDEWAYS_RIGHT);  
    else if (strcmp(direction, "left") == 0) moveMotors(g_pwm_left_rf, g_pwm_left_lf, g_pwm_left_rr, g_pwm_left_lr, MEC_SIDEWAYS_LEFT);  
    else stopMotors();  
}  
 
// =============================================================================  
//  OBSTACLE DETECTION  
// =============================================================================  
// Helper: returns minimum distance from two sensors (lower = more conservative)  
static float minOf2(gpio_num_t t1, gpio_num_t e1, gpio_num_t t2, gpio_num_t e2) {  
    float d1 = getStableDistance(t1, e1);  
    float d2 = getStableDistance(t2, e2);  
    return (d1 < d2) ? d1 : d2;  
}  
 
bool detectObstacle(const char* direction) {  
    float distance = 999.0f;  
    if      (strcmp(direction,"forward")==0)  
        distance = minOf2(TRIG_FRONTL, ECHO_FRONTL, TRIG_FRONTR, ECHO_FRONTR);  
    else if (strcmp(direction,"backward")==0)  
        distance = minOf2(TRIG_BACKL, ECHO_BACKL, TRIG_BACKR, ECHO_BACKR);  
    else if (strcmp(direction,"left")==0)  
        distance = getStableDistance(TRIG_LEFT,  ECHO_LEFT);  
    else if (strcmp(direction,"right")==0)  
        distance = getStableDistance(TRIG_RIGHT, ECHO_RIGHT);  
 
    bool blocked = (distance <= OBST_THRESH_CM);  
    ESP_LOGI(TAG, "    %s: %.1f cm%s", direction, distance, blocked ? " [BLOCKED]" : "");  
    return blocked;  
}  
 
void scanAndMapObstacles(vector<vector<int>>& g, int cx, int cy) {  
    ESP_LOGI(TAG, "Scanning obstacles from (%d, %d):", cx, cy);  
    bool hitFront = detectObstacle("forward"); vTaskDelay(pdMS_TO_TICKS(30));  
    bool hitRight = detectObstacle("right");   vTaskDelay(pdMS_TO_TICKS(30));  
    bool hitBack  = detectObstacle("backward");vTaskDelay(pdMS_TO_TICKS(30));  
    bool hitLeft  = detectObstacle("left");    vTaskDelay(pdMS_TO_TICKS(30));  
    int found = 0;  
    if (hitFront && cx+1 < (int)g.size())     { g[cx+1][cy]   = 1; ESP_LOGI(TAG,"  > FRONT"); found++; }  
    if (hitBack  && cx-1 >= 0)                { g[cx-1][cy]   = 1; ESP_LOGI(TAG,"  > BACK");  found++; }  
    if (hitLeft  && cy+1 < (int)g[cx].size()) { g[cx][cy+1]   = 1; ESP_LOGI(TAG,"  > LEFT");  found++; }  
    if (hitRight && cy-1 >= 0)                { g[cx][cy-1]   = 1; ESP_LOGI(TAG,"  > RIGHT"); found++; }  
    if (found == 0) ESP_LOGI(TAG,"  > No obstacles");  
}  
 
// ===========================================================================  
//  A* PATH PLANNING  
// =============================================================================  
struct Cell {  
    bool  closed = false;  
    float g = INFINITY, h = 0.0f, f = INFINITY;  
    int   parentX = -1, parentY = -1;  
};  
 
inline float heuristic(int x1, int y1, int x2, int y2) {  
    return (float)(abs(x1-x2) + abs(y1-y2));  
}  
 
vector<std::pair<int,int>> a_star_grid(const vector<vector<int>>& g,  
                                        int sx, int sy, int gx, int gy) {  
    int rows = (int)g.size();  
    int cols = (int)g[0].size();  
    if (gx<0||gy<0||gx>=rows||gy>=cols) return {};  
    if (sx<0||sy<0||sx>=rows||sy>=cols) return {};  
    if (g[gx][gy]==1) return {};  
 
    vector<vector<Cell>> map(rows, vector<Cell>(cols));  
    auto cmp = [](const std::tuple<float,int,int>& a, const std::tuple<float,int,int>& b){  
        return std::get<0>(a) > std::get<0>(b);  
    };  
    std::priority_queue<std::tuple<float,int,int>,  
                        vector<std::tuple<float,int,int>>,  
                        decltype(cmp)> open(cmp);  
 
    map[sx][sy].g = 0.0f;  
    map[sx][sy].h = heuristic(sx,sy,gx,gy);  
    map[sx][sy].f = map[sx][sy].h;  
    open.emplace(map[sx][sy].f, sx, sy);  
 
    const int dx[4] = {-1, 1, 0, 0};  
    const int dy[4] = { 0, 0,-1, 1};  
 
    while (!open.empty()) {  
        auto [cf, cx, cy] = open.top(); open.pop();  
        if (map[cx][cy].closed) continue;  
        map[cx][cy].closed = true;  
        if (cx==gx && cy==gy) {  
            vector<std::pair<int,int>> path;  
            int px=gx, py=gy;  
            while (!(px==sx && py==sy)) {  
                path.emplace_back(px,py);  
                int nx=map[px][py].parentX, ny=map[px][py].parentY;  
                if (nx<0||ny<0) break;  
                px=nx; py=ny;  
            }  
            path.emplace_back(sx,sy);  
            std::reverse(path.begin(), path.end());  
            return path;  
        }  
        for (int k=0;k<4;k++){  
            int nx=cx+dx[k], ny=cy+dy[k];  
            if (nx<0||ny<0||nx>=rows||ny>=(int)g[nx].size()) continue;  
            if (g[nx][ny]==1||map[nx][ny].closed) continue;  
            float tg = map[cx][cy].g + 1.0f;  
            if (tg < map[nx][ny].g) {  
                map[nx][ny].g = tg;  
                map[nx][ny].h = heuristic(nx,ny,gx,gy);  
                map[nx][ny].f = tg + map[nx][ny].h;  
                map[nx][ny].parentX = cx;  
                map[nx][ny].parentY = cy;  
                open.emplace(map[nx][ny].f, nx, ny);  
            }  
        }  
    }  
    return {};  
}  
 
// =============================================================================  
//  GRID UTILITIES  
// =============================================================================  
void shiftGrid(vector<vector<int>>& g, int shiftX, int shiftY) {  
    int rows=(int)g.size(), cols=(int)g[0].size();  
    vector<vector<int>> ng(rows, vector<int>(cols,0));  
    for (int r=0;r<rows;r++) for (int c=0;c<cols;c++){  
        int nr=r+shiftX, nc=c+shiftY;  
        if (nr>=0&&nr<rows&&nc>=0&&nc<cols) ng[nr][nc]=g[r][c];  
    }  
    g = ng;  
}  
 
void expandGrid(vector<vector<int>>& g, int expandBy) {  
    int oR=(int)g.size(), oC=(int)g[0].size();  
    int nR=min(oR+expandBy,MAX_GRID), nC=min(oC+expandBy,MAX_GRID);  
    if (nR<=oR && nC<=oC) return;  
    ESP_LOGI(TAG,"Expanding grid %dx%d -> %dx%d", oR, oC, nR, nC);  
    g.resize(nR);  
    for (int r=0;r<nR;r++) g[r].resize(nC, 0);  
    shiftGrid(g, expandBy/2, expandBy/2);  
}  
 
// =============================================================================  
//  SERIAL MAP PRINTER  
// =============================================================================  
void printMapToSerial() {  
    int rows=(int)grid.size(); if(rows==0) return;  
    int cols=(int)grid[0].size();  
    vector<vector<char>> disp(rows,vector<char>(cols,'.'));  
    for(int r=0;r<rows;r++) for(int c=0;c<cols;c++) if(grid[r][c]==1) disp[r][c]='X';  
    for(auto& p:visited) if(p.first>=0&&p.first<rows&&p.second>=0&&p.second<cols)  
        disp[p.first][p.second]='*';  
    if(curX>=0&&curX<rows&&curY>=0&&curY<cols) disp[curX][curY]='O';  
    if(origStartX>=0&&origStartX<rows&&origStartY>=0&&origStartY<cols)  
        disp[origStartX][origStartY]='S';  
    if(goalX>=0&&goalX<rows&&goalY>=0&&goalY<cols) disp[goalX][goalY]='G';  
 
    int minR=rows,maxR=-1,minC=cols,maxC=-1;  
    auto consider=[&](int r,int c){  
        minR=min(minR,r);maxR=max(maxR,r);minC=min(minC,c);maxC=max(maxC,c);  
    };  
    for(auto& p:visited) consider(p.first,p.second);  
    consider(curX,curY);
    if (origStartX >= 0 && origStartY >= 0) consider(origStartX,origStartY);
    if (goalX >= 0 && goalY >= 0) consider(goalX,goalY);
    for(int r=0;r<rows;r++) for(int c=0;c<cols;c++) if(disp[r][c]=='X') consider(r,c);  
    if(maxR==-1) return;  
 
    printf("\n======== CURRENT MAP ========\n");  
    printf("LEFT <-- Robot --> RIGHT\n\n");  
    for(int r=minR;r<=maxR;r++){  
        for(int c=maxC;c>=minC;c--) printf("%c ", disp[r][c]);  
        printf("\n");  
    }  
    printf("\nLegend: O=Robot S=Start G=Goal *=Path X=Obstacle .=Empty\n");  
    printf("=============================\n\n");  
}  
 
// =============================================================================  
//  MAP TO HTML  
// =============================================================================  
char* generateMapHTML() {  
    int rows=(int)grid.size();  
    if (rows==0) {  
        const char* msg="<div style='padding:20px;color:#999;'>Idle — press Chase Sound Source to begin.</div>";  
        char* buf=(char*)malloc(strlen(msg)+1); strcpy(buf,msg); return buf;  
    }  
    int cols=(int)grid[0].size();  
    vector<vector<char>> disp(rows,vector<char>(cols,'.'));  
    for(int r=0;r<rows;r++) for(int c=0;c<cols;c++) if(grid[r][c]==1) disp[r][c]='#';  
    for(auto& p:visited) if(p.first>=0&&p.first<rows&&p.second>=0&&p.second<cols)  
        disp[p.first][p.second]='P';  
    if(curX>=0&&curX<rows&&curY>=0&&curY<cols) disp[curX][curY]='R';  
    if(origStartX>=0&&origStartX<rows&&origStartY>=0&&origStartY<cols)  
        disp[origStartX][origStartY]='S';  
    if(goalX>=0&&goalX<rows&&goalY>=0&&goalY<cols) disp[goalX][goalY]='G';  
 
    int minR=rows,maxR=-1,minC=cols,maxC=-1;  
    auto consider=[&](int r,int c){  
        minR=min(minR,r);maxR=max(maxR,r);minC=min(minC,c);maxC=max(maxC,c);  
    };  
    for(auto& p:visited) consider(p.first,p.second);  
    consider(curX,curY);
    if (origStartX >= 0 && origStartY >= 0) consider(origStartX,origStartY);
    if (goalX >= 0 && goalY >= 0) consider(goalX,goalY);
    for(int r=0;r<rows;r++) for(int c=0;c<cols;c++) if(disp[r][c]=='#') consider(r,c);  
    if(maxR==-1){  
        const char* m="<div style='padding:20px;color:#999;'>Mapping...</div>";  
        char* b=(char*)malloc(strlen(m)+1); strcpy(b,m); return b;  
    }  
 
    size_t bufSize=4096+(size_t)(maxR-minR+1)*(maxC-minC+1)*120;  
    char* html=(char*)malloc(bufSize);  
    if(!html) return nullptr;  
    html[0]='\0';  
 
    strcat(html,"<div style='background:#1a1a1a;padding:15px;border-radius:10px;display:inline-block;'>");  
    strcat(html,"<div style='color:#00ff00;font-family:monospace;font-size:11px;line-height:1.3;text-align:left;white-space:pre;'>");  
    strcat(html,"LEFT &lt;-- Robot --&gt; RIGHT\n\n");  
    for(int r=minR;r<=maxR;r++){  
        for(int c=maxC;c>=minC;c--){  
            char ch=disp[r][c];  
            if      (ch=='R') strcat(html,"<span style='color:#00ffff;font-weight:bold;font-size:14px;'>O</span>");  
            else if (ch=='S') strcat(html,"<span style='color:#00ff00;font-weight:bold;'>S</span>");  
            else if (ch=='G') strcat(html,"<span style='color:#ff0000;font-weight:bold;'>G</span>");  
            else if (ch=='P') strcat(html,"<span style='color:#ffaa00;'>*</span>");  
            else if (ch=='#') strcat(html,"<span style='color:#ff3333;font-weight:bold;'>X</span>");  
            else              strcat(html,"<span style='color:#333;'>.</span>");  
        }  
        strcat(html,"\n");  
    }  
    strcat(html,"</div></div>");  
    return html;  
}  
 
// =============================================================================  
//  NAVIGATION INIT  (shared by sound-triggered and manual modes)
//
//  FIX 1: origStartX/Y is only written the very first time (g_orig_start_set
//  is false).  Mid-run replans and rechecks call this function again but
//  g_orig_start_set is already true, so the original start is preserved.
// =============================================================================  
void initializeNavigation(float direction_deg, float distance_in, bool from_sound) {  
    g_nav_start_time = esp_timer_get_time();
    g_nav_elapsed_us = 0;
    
    // FIX 2: reset distance & displacement counters only on a genuine fresh start
    if (!g_orig_start_set) {
        g_total_steps_traveled = 0;
        g_net_dx = 0;
        g_net_dy = 0;
    }

    printf("\n========================================\n");  
    printf("    NAVIGATION STARTED (%s)\n", from_sound ? "SOUND SOURCE" : "MANUAL");  
    printf("========================================\n");  
    printf("Direction: %.1f degrees\n", direction_deg);  
    printf("Distance:  %.1f inches\n",  distance_in);  
    printf("Recheck:   every %d steps\n", g_recheck_interval);  
    if (from_sound) {  
        printf("Start mag: %.0f  Arrival factor: %.1fx  Arrival obst: %.0f cm\n",  
               g_chase_start_mag, ARRIVAL_MAG_FACTOR, ARRIVAL_OBST_CM);  
    }  
    printf("========================================\n\n");  
 
    emergencyStop    = false;  
    navigationActive = true;  
    soundNavMode     = from_sound;  
    currentStep      = 0;
    s_arrival_confirm = 0;
    strncpy(g_current_dir, "forward", sizeof(g_current_dir));  
 
    int N = 30;  
    grid = vector<vector<int>>(N, vector<int>(N, 0));  
 
    int   startX = N/2, startY = N/2;  
    float node_size_in   = 9.0f;  
    float distance_nodes = distance_in / node_size_in;  
    float rad  = direction_deg * (float)M_PI / 180.0f;  
    float gdx  = distance_nodes * cosf(rad);  
    float gdy  = distance_nodes * -sinf(rad);  
 
    int goalX_initial = startX + (int)roundf(gdx);  
    int goalY_initial = startY + (int)roundf(gdy);  
 
    printf("Start: (%d,%d)  Goal: (%d,%d)\n", startX, startY, goalX_initial, goalY_initial);  
 
    int shiftX = (min(startX, goalX_initial) < 0) ? -min(startX, goalX_initial) : 0;  
    int shiftY = (min(startY, goalY_initial) < 0) ? -min(startY, goalY_initial) : 0;  
    if (shiftX||shiftY) {  
        shiftGrid(grid, shiftX, shiftY);  
        printf("Grid shifted (%d,%d)\n", shiftX, shiftY);  
    }  
 
    curX = startX + shiftX;  curY = startY + shiftY;  
    goalX = goalX_initial + shiftX; goalY = goalY_initial + shiftY;

    // FIX 1: Only set the true original start once.  Replans must not move it.
    if (!g_orig_start_set) {
        origStartX = curX; origStartY = curY;
        g_orig_start_set = true;
        // Also record the absolute start for return-to-origin (grid-shift-proof)
        g_abs_start_grid_x = curX;
        g_abs_start_grid_y = curY;
        g_abs_start_valid  = true;
    }
 
    visited.clear();  
    visited.push_back({curX, curY});  
 
    int needR = min(max(goalX+3, N), MAX_GRID);  
    int needC = min(max(goalY+3, N), MAX_GRID);  
    grid.resize(needR);  
    for(int r=0;r<needR;r++) grid[r].resize(needC, 0);  
 
    printf("Grid: %d x %d\n\nInitial scan...\n", needR, needC);  
    snprintf(statusMessage, sizeof(statusMessage),  
             from_sound ? "Sound nav init..." : "Manual nav init...");  
 
    scanAndMapObstacles(grid, curX, curY);  
    vTaskDelay(pdMS_TO_TICKS(200));  
    printMapToSerial();  
 
    snprintf(statusMessage, sizeof(statusMessage),  
             from_sound ? "Chasing sound %.1f deg" : "Navigation active", direction_deg);  
    setLED(true);  
    printf("Navigation initialized!\n\n");  
}  
 
// =============================================================================  
//  WIFI ACCESS POINT  
// =============================================================================  
static void wifi_event_handler(void* arg, esp_event_base_t eb, int32_t id, void* data) {  
    if (eb==WIFI_EVENT&&id==WIFI_EVENT_AP_STACONNECTED)    ESP_LOGI(TAG,"Station connected");  
    if (eb==WIFI_EVENT&&id==WIFI_EVENT_AP_STADISCONNECTED) ESP_LOGI(TAG,"Station disconnected");  
}  
 
void setupAP() {  
    printf("\n========================================\n");  
    printf("Starting WiFi Access Point...\n");  
    esp_netif_init();  
    esp_event_loop_create_default();  
    esp_netif_create_default_wifi_ap();  
 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  
    esp_wifi_init(&cfg);  
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  
                                        &wifi_event_handler, NULL, NULL);  
 
    wifi_config_t wc = {};  
    strncpy((char*)wc.ap.ssid, "Sound_Rover", sizeof(wc.ap.ssid));  
    wc.ap.ssid_len       = strlen("Sound_Rover");  
    wc.ap.channel        = 1;  
    wc.ap.max_connection  = 4;  
    wc.ap.authmode       = WIFI_AUTH_OPEN;  
 
    esp_wifi_set_mode(WIFI_MODE_AP);  
    esp_wifi_set_config(WIFI_IF_AP, &wc);  
    esp_wifi_start();  
 
    printf("AP: Sound_Rover  IP: 192.168.4.1\n");  
    printf("========================================\n\n");  
    for(int i=0;i<6;i++){ setLED(true); vTaskDelay(pdMS_TO_TICKS(200)); setLED(false); vTaskDelay(pdMS_TO_TICKS(200)); }  
    setLED(true);  
}  
 
// =============================================================================  
//  WEB PAGE
//
//  FIX 3: Added "Session" panel at the bottom of the side column with:
//    - pageTimer: HH:MM:SS wall-clock since the browser tab was opened
//    - distLabel: total distance traveled, extracted from the /status string
//    - netXYLabel: Net X and Y physical displacement since start
// =============================================================================  
static const char webpage[] =  
"<!DOCTYPE html><html><head>"  
"<meta charset='UTF-8'>"  
"<meta name='viewport' content='width=device-width,initial-scale=1.0'>"  
"<title>Sound Rover</title>"  
"<style>"  
"body{font-family:'Segoe UI',sans-serif;"  
"background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);"  
"margin:0;text-align:center;min-height:100vh;}"  
"h1{margin-top:30px;font-size:42px;color:white;text-shadow:2px 2px 4px rgba(0,0,0,0.3);}"  
".container{display:flex;justify-content:center;gap:30px;margin:30px auto;"  
"flex-wrap:wrap;max-width:1500px;}"  
".map-box{width:700px;min-height:500px;background:#2a2a2a;border-radius:20px;"  
"box-shadow:0 8px 32px rgba(0,0,0,0.3);padding:30px;"  
"display:flex;justify-content:center;align-items:center;}"  
".side{display:flex;flex-direction:column;gap:14px;width:300px;}"  
".info-box{background:white;border-radius:15px;box-shadow:0 4px 20px rgba(0,0,0,0.2);padding:15px;}"  
".info-box h3{margin-top:0;color:#667eea;}"  
".info-box p,.info-box label{margin:5px 0;text-align:left;font-size:14px;display:block;}"  
".info-box input{width:calc(100% - 20px);padding:8px;margin:4px 0;"  
"border:2px solid #ddd;border-radius:8px;font-size:14px;}"  
"button{font-size:17px;padding:13px;border:none;border-radius:12px;cursor:pointer;"  
"color:white;width:100%;font-weight:bold;text-transform:uppercase;"  
"box-shadow:0 4px 15px rgba(0,0,0,0.2);transition:all 0.3s;margin:3px 0;}"  
"button:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(0,0,0,0.3);}"  
".btn-start{background:linear-gradient(135deg,#667eea,#764ba2);}"  
".btn-stop{background:linear-gradient(135deg,#f093fb,#f5576c);}"  
".btn-sound{background:linear-gradient(135deg,#11998e,#38ef7d);color:#000;}"  
".btn-cfg{background:linear-gradient(135deg,#f7971e,#ffd200);color:#000;}"  
".btn-return{background:linear-gradient(135deg,#667eea,#43cea2);}"  
"#status{font-weight:bold;color:#667eea;font-size:15px;padding:10px;"  
"background:#f0f4ff;border-radius:10px;margin-top:6px;}"  
"#arrivalBanner{display:none;font-weight:bold;font-size:16px;padding:12px;"  
"background:linear-gradient(135deg,#11998e,#38ef7d);color:#000;"  
"border-radius:10px;margin-top:6px;text-align:center;animation:pulse 1s ease-in-out infinite alternate;}"  
"@keyframes pulse{from{opacity:1}to{opacity:0.7}}"  
"#soundInfo{font-size:13px;font-family:monospace;padding:10px;"  
"background:#1a1a1a;color:#00ff00;border-radius:8px;text-align:left;line-height:1.7;}"  
".legend{text-align:left;font-size:13px;background:#1a1a1a;padding:12px;"  
"border-radius:10px;color:#aaa;font-family:monospace;}"  
".legend span{display:block;margin:4px 0;}"  
"</style>"  
"<script>"
"var _pageStart=Date.now(),_arrived=false;"
"function fmtTime(s){"
"var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;"
"return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(sec).padStart(2,'0');}"
"setInterval(function(){"
"var s=Math.floor((Date.now()-_pageStart)/1000);"
"document.getElementById('pageTimer').textContent='Page up: '+fmtTime(s);"
"},1000);"
"function upd(){"
"fetch('/map').then(r=>r.text()).then(d=>{document.getElementById('mapDisplay').innerHTML=d;}).catch(()=>{});"
"fetch('/status').then(r=>r.text()).then(function(d){"
"document.getElementById('status').textContent=d;"
"var m=d.match(/Dist:([.0-9]+)in/);"
"document.getElementById('distLabel').textContent=m"
"?'Distance traveled: '+m[1]+' in ('+(parseFloat(m[1])/12).toFixed(1)+' ft)'"
":'Distance traveled: 0 in';"
"var mNet=d.match(/Net:(-?\\d+),(-?\\d+)/);"
"var nLab=document.getElementById('netXYLabel');"
"if(nLab){ nLab.textContent=mNet?'Net displacement: X='+(mNet[1]*9)+'in, Y='+(mNet[2]*9)+'in':'Net displacement: 0in, 0in'; }"
"var isArrived=d.indexOf('Arrived at sound source')>=0||d.indexOf('Grid goal reached')>=0;"
"var banner=document.getElementById('arrivalBanner');"
"if(isArrived&&!_arrived){_arrived=true;banner.style.display='block';}"
"else if(!isArrived&&!_arrived){banner.style.display='none';}"
"}).catch(()=>{});"
"fetch('/soundinfo').then(r=>r.text()).then(d=>{document.getElementById('soundInfo').innerHTML=d;}).catch(()=>{});}"
"function clearArrival(){"
"_arrived=false;"
"document.getElementById('arrivalBanner').style.display='none';"
"document.getElementById('mapDisplay').innerHTML='<div style=\"color:#666;\">Starting...</div>';}"
"function startNav(){"
"clearArrival();"
"var d=document.getElementById('direction').value;"
"var dist=document.getElementById('distance').value;"
"fetch('/start?dir='+d+'&dist='+dist).then(r=>r.text()).then(d=>{alert(d);setTimeout(upd,500);});}"
"function stopNav(){"
"fetch('/stop').then(r=>r.text()).then(d=>{alert(d);setTimeout(upd,500);});}"
"function startSoundNav(){"
"clearArrival();"
"fetch('/sound_start').then(r=>r.text()).then(d=>{alert(d);setTimeout(upd,800);});}"
"function returnStart(){"
"fetch('/return_start').then(r=>r.text()).then(d=>{alert(d);setTimeout(upd,500);});}"
"function setRecheck(){"
"var n=document.getElementById('recheck').value;"
"fetch('/set_recheck?n='+n).then(r=>r.text()).then(d=>alert(d));}"
"function setFreq(){"
"var f=document.getElementById('targetFreq').value;"
"fetch('/set_freq?f='+f).then(r=>r.text()).then(d=>alert(d));}"
"function setMoveDelay(){"
"var fwd=document.getElementById('delayFwd').value;"
"var side=document.getElementById('delaySide').value;"
"fetch('/set_move_delay?fwd='+fwd+'&side='+side).then(r=>r.text()).then(d=>alert(d));}"
"function setStrafe(){"
"var l=document.getElementById('pwmLeft').value;"
"var r=document.getElementById('pwmRight').value;"
"fetch('/set_strafe?l='+l+'&r='+r).then(res=>res.text()).then(d=>alert(d));}"
"upd();setInterval(upd,1000);"
"</script>"
"</head><body>"
"<h1>Sound Rover</h1>"
"<div class='container'>"
"<div class='map-box' id='mapDisplay'>"
"<div style='color:#666;'>Idle — press Chase Sound Source to begin.</div>"
"</div>"
"<div class='side'>"
"<div id='arrivalBanner'>&#10003; ARRIVED AT SOUND SOURCE!</div>"
"<div class='info-box'><h3>Live Sound Detection</h3>"
"<div id='soundInfo'>Initializing...</div></div>"
"<div class='info-box'><h3>Manual Navigation</h3>"
"<label>Direction (0-360 deg):</label>"
"<input type='number' id='direction' value='90' min='0' max='360'>"
"<label>Distance (inches):</label>"
"<input type='number' id='distance' value='126' min='1' max='500'>"
"</div>"
"<div class='info-box'><h3>Sound Recheck</h3>"
"<label>Re-check every N steps (0=off):</label>"
"<input type='number' id='recheck' value='5' min='0' max='100'>"
"<button class='btn-cfg' onclick='setRecheck()'>Set Recheck Interval</button>"
"</div>"
"<div class='info-box'><h3>Sound Config</h3>"
"<label>Target Frequency (Hz):</label>"
"<input type='number' id='targetFreq' value='1000' min='100' max='1500' step='50'>"
"<button class='btn-cfg' onclick='setFreq()'>Set Target Freq</button>"
"</div>"
"<div class='info-box'><h3>Move Timing</h3>"
"<label>Forward / Backward delay (ms):</label>"
"<input type='number' id='delayFwd' value='235' min='50' max='2000'>"
"<label>Left / Right strafe delay (ms):</label>"
"<input type='number' id='delaySide' value='235' min='50' max='2000'>"
"<button class='btn-cfg' onclick='setMoveDelay()'>Set Move Delays</button>"
"</div>"
"<div class='info-box'><h3>Strafe Motor PWMs</h3>"
"<div style='display:flex;gap:5px;'>"
"<div style='flex:1;'><label style='font-size:12px;'>Left (RF,LF,RR,LR):</label>"
"<input type='text' id='pwmLeft' value='229,235,237,230'></div>"
"<div style='flex:1;'><label style='font-size:12px;'>Right (RF,LF,RR,LR):</label>"
"<input type='text' id='pwmRight' value='230,240,240,230'></div>"
"</div>"
"<button class='btn-cfg' onclick='setStrafe()'>Set Strafe PWM</button>"
"</div>"
"<div class='info-box'><h3>Status</h3><div id='status'>Ready</div></div>"
"<button class='btn-sound' onclick='startSoundNav()'>Chase Sound Source</button>"
"<button class='btn-start' onclick='startNav()'>Manual Start</button>"
"<button class='btn-return' onclick='returnStart()'>Return to Start</button>"
"<button class='btn-stop' onclick='stopNav()'>Emergency Stop</button>"  
"<div class='legend'>"  
"<h3 style='color:#00ff00;margin-top:0;'>Map Legend</h3>"  
"<span style='color:#00ffff;'>O - Robot Position</span>"  
"<span style='color:#00ff00;'>S - Start Point</span>"  
"<span style='color:#ff0000;'>G - Goal (Sound)</span>"  
"<span style='color:#ffaa00;'>* - Path Traveled</span>"  
"<span style='color:#ff3333;'>X - Obstacles</span>"  
"</div>"
"<div class='info-box' style='margin-top:4px;'>"
"<h3>Session</h3>"
"<div id='pageTimer' style='font-family:monospace;font-size:14px;'>Page up: 00:00:00</div>"
"<div id='distLabel' style='font-family:monospace;font-size:14px;margin-top:6px;'>Distance traveled: 0 in</div>"
"<div id='netXYLabel' style='font-family:monospace;font-size:14px;margin-top:6px;color:#667eea;'>Net displacement: 0in, 0in</div>"
"</div>"
"</div></div></body></html>";  
 
// =============================================================================  
//  HTTP HANDLERS  
// =============================================================================  
static esp_err_t handler_root(httpd_req_t* req) {  
    httpd_resp_set_type(req, "text/html");  
    httpd_resp_send(req, webpage, strlen(webpage));  
    return ESP_OK;  
}  
 
static esp_err_t handler_map(httpd_req_t* req) {  
    char* html = generateMapHTML();  
    httpd_resp_set_type(req, "text/html");  
    httpd_resp_send(req, html ? html : "", html ? strlen(html) : 0);  
    if (html) free(html);  
    return ESP_OK;  
}  
 
static esp_err_t handler_status(httpd_req_t* req) {  
    char buf[512];  
    if (navigationActive) {  
        const char* mode = soundNavMode ? "SoundNav" : "ManualNav";  
        int elapsed_sec = (int)(g_nav_elapsed_us / 1000000);  
        int minutes = elapsed_sec / 60;  
        int seconds = elapsed_sec % 60;
        float dist_traveled_in = g_total_steps_traveled * 9.0f;
        // Fix 2: emit net displacement robust to grid-shifts
        snprintf(buf, sizeof(buf),  
                 "[%s] %s | Step:%d | Pos:(%d,%d) | ToGoal:%.0f | Time:%02d:%02d | Recheck:%d | Dist:%.0fin | Net:%d,%d",  
                 mode, statusMessage, currentStep, curX, curY,  
                 heuristic(curX,curY,goalX,goalY), minutes, seconds, g_recheck_interval,
                 dist_traveled_in, g_net_dx, g_net_dy);
    } else {  
        if (g_nav_elapsed_us > 0) {  
            int elapsed_sec = (int)(g_nav_elapsed_us / 1000000);  
            int minutes = elapsed_sec / 60;  
            int seconds = elapsed_sec % 60;
            float dist_traveled_in2 = g_total_steps_traveled * 9.0f;
            snprintf(buf, sizeof(buf), "%s | Last run: %02d:%02d | Dist:%.0fin | Net:%d,%d",  
                     statusMessage, minutes, seconds, dist_traveled_in2, g_net_dx, g_net_dy);
        } else {  
            snprintf(buf, sizeof(buf), "%s | Net:%d,%d", statusMessage, g_net_dx, g_net_dy);  
        }  
    }  
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, buf, strlen(buf));  
    return ESP_OK;  
}  
 
static esp_err_t handler_soundinfo(httpd_req_t* req) {  
    char buf[512];  
    float ang=0, dist=-1, mag=0;  
    bool  valid=false;  
    float rms[4]={};  
    if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {  
        ang=g_sound_angle; dist=g_sound_distance; valid=g_sound_valid;  
        mag=g_sound_mag;  
        for(int m=0;m<4;m++) rms[m]=g_sound_rms[m];  
        xSemaphoreGive(g_sound_mutex);  
    }  
    if (valid) {  
        snprintf(buf, sizeof(buf),  
                 "<b>%.1f deg</b> | %.3fm<br>"  
                 "M0:%.4f M1:%.4f<br>M2:%.4f M3:%.4f<br>"  
                 "Mag:%.0f  StartMag:%.0f<br>"  
                 "Target: %.0fHz +/-%.0fHz",  
                 ang, dist, rms[0], rms[1], rms[2], rms[3],  
                 mag, g_chase_start_mag,  
                 g_target_freq, TARGET_SIGMA);  
    } else {  
        snprintf(buf, sizeof(buf),  
                 "(Quiet — listening...)<br>"  
                 "M0:%.4f M1:%.4f<br>M2:%.4f M3:%.4f<br>"  
                 "Target: %.0fHz",  
                 rms[0], rms[1], rms[2], rms[3], g_target_freq);  
    }  
    httpd_resp_set_type(req, "text/html");  
    httpd_resp_send(req, buf, strlen(buf));  
    return ESP_OK;  
}  
 
static esp_err_t handler_start(httpd_req_t* req) {  
    char query[128]={};  
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {  
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query"); return ESP_FAIL;  
    }  
    char dir_s[32]={}, dist_s[32]={};  
    httpd_query_key_value(query, "dir",  dir_s,  sizeof(dir_s));  
    httpd_query_key_value(query, "dist", dist_s, sizeof(dist_s));
    g_orig_start_set = false;  // treat manual start as a fresh session
    initializeNavigation(atof(dir_s), atof(dist_s), false);  
    const char* resp = "Manual navigation started!";  
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, resp, strlen(resp));  
    return ESP_OK;  
}  
 
static esp_err_t handler_sound_start(httpd_req_t* req) {  
    float ang=0, dist=-1, mag=0;  
    bool  valid=false;  
    if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {  
        ang=g_sound_angle; dist=g_sound_distance; valid=g_sound_valid;  
        mag=g_sound_mag;  
        xSemaphoreGive(g_sound_mutex);  
    }  
    char resp[256];  
    if (!valid) {  
        snprintf(resp, sizeof(resp),  
                 "No sound detected! Ensure a %.0fHz tone is audible.", g_target_freq);  
    } else {  
        // Use the actual measured magnitude as the baseline.
        // Clamping to MAG_THRESHOLD was wrong: it inflated the baseline when
        // the source was loud, making the arrival threshold unreachably high
        // (4x a value already at MAG_THRESHOLD), causing the robot to stop far
        // too early OR never arrive depending on room acoustics.
        g_chase_start_mag = mag;
        printf("\n[SoundStart] Baseline magnitude: %.0f\n", g_chase_start_mag);  
        printf("[SoundStart] Arrival: mag >= %.0f AND obstacle <= %.0f cm\n",  
               g_chase_start_mag * ARRIVAL_MAG_FACTOR, ARRIVAL_OBST_CM);
        // Clear any prior emergency stop so the nav_task polling loop can
        // actually consume the chase request (it skips the block when
        // emergencyStop is true, making Chase a no-op after a stop).
        emergencyStop     = false;
        g_orig_start_set = false;  // fresh chase session — reset origin lock

        if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {  
            g_sound_updated   = true;  
            g_chase_requested = true;  
            xSemaphoreGive(g_sound_mutex);  
        }  
 
        float dist_in = (dist > 0) ? dist * 39.37f : SOUND_MIN_DISTANCE_IN;  
        if (dist_in < SOUND_MIN_DISTANCE_IN) dist_in = SOUND_MIN_DISTANCE_IN;  
        if (dist_in > 500.0f) dist_in = 500.0f;  
        snprintf(resp, sizeof(resp),  
                 "Chasing sound! Angle=%.1f deg  Dist=%.1fin (%.2fm)  StartMag=%.0f",  
                 ang, dist_in, dist, g_chase_start_mag);  
    }  
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, resp, strlen(resp));  
    return ESP_OK;  
}  
 
static esp_err_t handler_return_start(httpd_req_t* req) {  
    char resp[256];  
   
    if (navigationActive) {  
        snprintf(resp, sizeof(resp),  
                 "Navigation already active. Stop first, then return to start.");  
        httpd_resp_set_type(req, "text/plain");  
        httpd_resp_send(req, resp, strlen(resp));  
        return ESP_OK;  
    }  
   
    if (!g_orig_start_set) {  
        snprintf(resp, sizeof(resp),  
                 "No start position recorded. Navigate somewhere first!");  
        httpd_resp_set_type(req, "text/plain");  
        httpd_resp_send(req, resp, strlen(resp));  
        return ESP_OK;  
    }  
   
    if (g_net_dx == 0 && g_net_dy == 0) {  
        snprintf(resp, sizeof(resp), "Already at start position!");  
        httpd_resp_set_type(req, "text/plain");  
        httpd_resp_send(req, resp, strlen(resp));  
        return ESP_OK;  
    }  
   
    // We want to return exactly -g_net_dx and -g_net_dy relative to where we are now.
    // This is entirely immune to whatever grid expansions/shifts happened.
    int tgt_dx = -g_net_dx;
    int tgt_dy = -g_net_dy;

    ESP_LOGI(TAG, "Return to start: current net is (%d,%d), target dx,dy is (%d,%d)",  
             g_net_dx, g_net_dy, tgt_dx, tgt_dy);  
   
    // atan2f expects (-dy, dx) map logic. 
    // In our coordinate space tgt_dx acts as dx, tgt_dy acts as dy.
    float angle_rad = atan2f(-tgt_dy, tgt_dx);  
    float angle_deg = angle_rad * 180.0f / M_PI;  
    if (angle_deg < 0) angle_deg += 360.0f;  
   
    float distance_nodes = sqrtf((float)(tgt_dx*tgt_dx + tgt_dy*tgt_dy));  
    float distance_in = distance_nodes * 9.0f;  
    if (distance_in < 9.0f) distance_in = 9.0f;

    printf("Return navigation: %.1f deg, %.1f inches to origin\n", angle_deg, distance_in);
   
    // Initialize standard navigation to the targeted offset
    g_orig_start_set  = false;
    g_abs_start_valid = false;
    initializeNavigation(angle_deg, distance_in, false);

    // Override the goal to enforce exact cellular grid mapping 
    // (prevents math roundoff errors drifting the goal cell by 1)
    goalX = curX + tgt_dx;
    goalY = curY + tgt_dy;
    g_abs_start_grid_x = goalX;
    g_abs_start_grid_y = goalY;
    g_abs_start_valid  = true;

    snprintf(statusMessage, sizeof(statusMessage), "Returning to start...");

    snprintf(resp, sizeof(resp),  
             "Returning to start! Angle=%.1f deg Distance=%.1f in",  
             angle_deg, distance_in);  
   
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, resp, strlen(resp));  
    return ESP_OK;  
}  
 
static esp_err_t handler_set_recheck(httpd_req_t* req) {  
    char query[64]={};  
    httpd_req_get_url_query_str(req, query, sizeof(query));  
    char n_s[16]={};  
    httpd_query_key_value(query, "n", n_s, sizeof(n_s));  
    int n = atoi(n_s);  
    if (n < 0) n = 0;  
    g_recheck_interval = n;  
    char resp[64];  
    snprintf(resp, sizeof(resp), "Recheck interval: %d steps", n);  
    printf("[Config] %s\n", resp);  
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, resp, strlen(resp));  
    return ESP_OK;  
}  
 
static esp_err_t handler_stop(httpd_req_t* req) {  
    ESP_LOGI(TAG, "\n!!! EMERGENCY STOP !!!\n");  
    stopMotors();  
    emergencyStop          = true;  
    navigationActive       = false;  
    soundNavMode           = false;  
    g_chase_requested      = false;
    // FIX 1: allow a new fresh origin to be recorded on the next Chase press
    g_orig_start_set       = false;
    // FIX 2: reset distance so the counter starts clean on the next run
    g_total_steps_traveled = 0;
    snprintf(statusMessage, sizeof(statusMessage),  
             "EMERGENCY STOP — press Chase Sound Source to resume.");  
    setLED(true);  
    const char* resp = "STOPPED";  
    httpd_resp_set_type(req, "text/plain");  
    httpd_resp_send(req, resp, strlen(resp));  
    return ESP_OK;  
}  
 
static esp_err_t handler_set_freq(httpd_req_t* req) {
    char query[64]={};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char f_s[16]={};
    httpd_query_key_value(query, "f", f_s, sizeof(f_s));
    float f = atof(f_s);
    char resp[64];
    if (f >= 100.0f && f <= 8000.0f) {
        g_target_freq = f;
        // DSP tables are rebuilt automatically by sound_task on next loop tick
        printf("[Config] Target freq -> %.0f Hz (DSP will rebuild)\n", g_target_freq);
        snprintf(resp, sizeof(resp), "Target freq set: %.0f Hz", g_target_freq);
    } else {
        snprintf(resp, sizeof(resp), "Invalid freq (100-8000 Hz). Current: %.0f Hz", g_target_freq);
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t handler_set_move_delay(httpd_req_t* req) {
    char query[64]={};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char fwd_s[16]={}, side_s[16]={};
    httpd_query_key_value(query, "fwd",  fwd_s,  sizeof(fwd_s));
    httpd_query_key_value(query, "side", side_s, sizeof(side_s));
    if (strlen(fwd_s)  > 0) { int v = atoi(fwd_s);  if (v >= 50 && v <= 2000) g_move_delay_fwd_ms  = v; }
    if (strlen(side_s) > 0) { int v = atoi(side_s); if (v >= 50 && v <= 2000) g_move_delay_side_ms = v; }
    char resp[64];
    snprintf(resp, sizeof(resp), "Move delay: fwd=%dms side=%dms",
             g_move_delay_fwd_ms, g_move_delay_side_ms);
    printf("[Config] %s\n", resp);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

static esp_err_t handler_set_strafe(httpd_req_t* req) {
    char query[128]={};
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char l_s[32]={}, r_s[32]={};
    httpd_query_key_value(query, "l", l_s, sizeof(l_s));
    httpd_query_key_value(query, "r", r_s, sizeof(r_s));

    if (strlen(l_s) > 0) {
        sscanf(l_s, "%d,%d,%d,%d", &g_pwm_left_rf, &g_pwm_left_lf, &g_pwm_left_rr, &g_pwm_left_lr);
    }
    if (strlen(r_s) > 0) {
        sscanf(r_s, "%d,%d,%d,%d", &g_pwm_right_rf, &g_pwm_right_lf, &g_pwm_right_rr, &g_pwm_right_lr);
    }

    char resp[128];
    snprintf(resp, sizeof(resp), "Strafe PWM updated:\nLeft: %d,%d,%d,%d\nRight: %d,%d,%d,%d",
             g_pwm_left_rf, g_pwm_left_lf, g_pwm_left_rr, g_pwm_left_lr,
             g_pwm_right_rf, g_pwm_right_lf, g_pwm_right_rr, g_pwm_right_lr);
    printf("[Config] %s\n", resp);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

httpd_handle_t start_webserver() {  
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();  
    config.max_uri_handlers = 15;  
    httpd_handle_t server = NULL;  
    ESP_ERROR_CHECK(httpd_start(&server, &config));  
 
    httpd_uri_t uris[] = {  
        { "/",                HTTP_GET, handler_root,            NULL },  
        { "/map",             HTTP_GET, handler_map,             NULL },  
        { "/status",          HTTP_GET, handler_status,          NULL },  
        { "/soundinfo",       HTTP_GET, handler_soundinfo,       NULL },  
        { "/start",           HTTP_GET, handler_start,           NULL },  
        { "/sound_start",     HTTP_GET, handler_sound_start,     NULL },  
        { "/set_recheck",     HTTP_GET, handler_set_recheck,     NULL },  
        { "/set_freq",        HTTP_GET, handler_set_freq,        NULL },  
        { "/set_move_delay",  HTTP_GET, handler_set_move_delay,  NULL },  
        { "/set_strafe",      HTTP_GET, handler_set_strafe,      NULL },  
        { "/return_start",    HTTP_GET, handler_return_start,    NULL },  
        { "/stop",            HTTP_GET, handler_stop,            NULL },  
    };  
    for (auto& u : uris) httpd_register_uri_handler(server, &u);  
    return server;  
}  
 
// =============================================================================  
//  NAVIGATION TASK  (Core 1)  
// =============================================================================  
void nav_task(void* param) {  
    while (true) {  
 
        if (navigationActive && g_nav_start_time > 0) {  
            g_nav_elapsed_us = esp_timer_get_time() - g_nav_start_time;  
        }  
 
        if (!navigationActive) {  
            if (!emergencyStop) {  
                bool  chase  = false;  
                float ang    = 0.0f;  
                float dist_m = -1.0f;  
 
                if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {  
                    if (g_chase_requested && g_sound_valid) {  
                        chase             = true;  
                        ang               = g_sound_angle;  
                        dist_m            = g_sound_distance;  
                        g_chase_requested = false;  
                        g_sound_updated   = false;  
                    }  
                    xSemaphoreGive(g_sound_mutex);  
                }  
 
                if (chase) {  
                    printf("\n[Nav] Chase requested! Angle=%.1f deg Dist=%.3fm  StartMag=%.0f\n",  
                           ang, dist_m, g_chase_start_mag);  
                    float dist_in = (dist_m > 0) ? dist_m * 39.37f : SOUND_MIN_DISTANCE_IN;  
                    if (dist_in < SOUND_MIN_DISTANCE_IN) dist_in = SOUND_MIN_DISTANCE_IN;  
                    if (dist_in > 500.0f) dist_in = 500.0f;  
                    initializeNavigation(ang, dist_in, true);  
                }  
            }  
 
            if (!navigationActive) {  
                snprintf(statusMessage, sizeof(statusMessage),  
                         emergencyStop  
                             ? "STOPPED — press Chase Sound Source to resume."  
                             : "Idle — press Chase Sound Source to begin.");  
            }  
            vTaskDelay(pdMS_TO_TICKS(50));  
            continue;  
        }  
 
        if (curX==goalX && curY==goalY) {  
            printf("\n****************************************\n");  
            printf("  GOAL REACHED!  Steps:%d  Mode:%s\n",  
                   currentStep, soundNavMode ? "Sound" : "Manual");  
            printf("  Final pos: (%d,%d)\n", curX, curY);  
            printf("****************************************\n\n");  
            printMapToSerial();  
            snprintf(statusMessage, sizeof(statusMessage),  
                     soundNavMode  
                         ? "Grid goal reached! Press Chase to follow again."  
                         : "Goal reached!");  
            navigationActive = false;  
            soundNavMode     = false;  
            if (xSemaphoreTake(g_sound_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {  
                g_sound_updated   = false;  
                g_chase_requested = false;  
                xSemaphoreGive(g_sound_mutex);  
            }  
            for(int i=0;i<5;i++){ setLED(true); vTaskDelay(pdMS_TO_TICKS(200)); setLED(false); vTaskDelay(pdMS_TO_TICKS(200)); }  
            setLED(true);  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        if (currentStep > MAX_STEPS) {  
            printf("\n*** STEP LIMIT REACHED ***\n\n");  
            snprintf(statusMessage, sizeof(statusMessage),  
                     "Step limit — press Chase Sound Source to retry.");  
            navigationActive = false;  
            setLED(true);  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        if (soundNavMode && g_recheck_interval > 0  
            && currentStep > 0 && (currentStep % g_recheck_interval) == 0) {  
 
            printf("\n[Nav] Step %d: sound recheck\n", currentStep);  
            stopMotors();  
            vTaskDelay(pdMS_TO_TICKS(150));  
 
            float new_ang = 0.0f, new_dist = -1.0f;  
            bool  got     = recheckSound(new_ang, new_dist);  
 
            if (got) {  
                float cur_goal_dir = atan2f(-((float)(goalY-curY)), (float)(goalX-curX)) * 180.0f / PI_F;                if (cur_goal_dir < 0) cur_goal_dir += 360.0f;  
                float delta = fabsf(new_ang - cur_goal_dir);  
                if (delta > 180.0f) delta = 360.0f - delta;  
 
                printf("[Nav] Recheck: new=%.1f deg  cur_dir=%.1f deg  delta=%.1f deg\n",  
                       new_ang, cur_goal_dir, delta);  
 
                if (delta > RECHECK_ANGLE_DELTA) {  
                    printf("[Nav] Heading changed %.1f deg > %.1f deg — RE-PLANNING\n",  
                           delta, RECHECK_ANGLE_DELTA);  
                    float remaining_in = heuristic(curX, curY, goalX, goalY) * 9.0f;  
                    if (remaining_in < 9.0f) remaining_in = 9.0f;  
                    float dist_in = (new_dist > 0) ? new_dist * 39.37f : remaining_in;  
                    if (dist_in < remaining_in) dist_in = remaining_in;  
                    if (dist_in > 500.0f) dist_in = 500.0f;  
                    printf("[Nav] Replan dist=%.1fin (remaining=%.1fin)\n", dist_in, remaining_in);
                    // g_orig_start_set remains true here — replan preserves origin
                    initializeNavigation(new_ang, dist_in, true);  
                    continue;  
                } else {  
                    printf("[Nav] Heading stable — continuing\n");  
                    snprintf(statusMessage, sizeof(statusMessage),  
                             "Recheck OK (%.1f deg) — continuing", new_ang);  
                }  
            }  
        }  
 
        printf("----------------------------------------\n");  
        printf("STEP %d/%d  Pos:(%d,%d) Goal:(%d,%d) [%s]\n",  
               currentStep+1, MAX_STEPS, curX, curY, goalX, goalY,  
               soundNavMode ? "Sound" : "Manual");  
 
        auto path = a_star_grid(grid, curX, curY, goalX, goalY);  
 
        if (path.empty()) {  
            printf("No path found!\n");  
            if ((int)grid.size()<MAX_GRID || (int)grid[0].size()<MAX_GRID) {  
                printf("Expanding grid...\n");  
                expandGrid(grid, EXPAND_BY);  
                vTaskDelay(pdMS_TO_TICKS(100));  
                continue;  
            }  
            printf("\n*** PATH BLOCKED ***\n\n");  
            snprintf(statusMessage, sizeof(statusMessage),  
                     "Path blocked — press Chase Sound Source to retry.");  
            navigationActive = false;  
            setLED(true);  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        printf("Path: %d waypoints\n", (int)path.size());  
 
        if ((int)path.size() < 2) {  
            snprintf(statusMessage, sizeof(statusMessage),  
                     "Already at goal — press Chase Sound Source to retry.");  
            navigationActive = false;  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        int nx=path[1].first, ny=path[1].second;  
        int ddx=nx-curX, ddy=ny-curY;  
 
        const char* dir="stop";  
        if      (ddx== 1) dir="forward";  
        else if (ddx==-1) dir="backward";  
        else if (ddy== 1) dir="left";  
        else if (ddy==-1) dir="right";  
 
        strncpy(g_current_dir, dir, sizeof(g_current_dir)-1);  
        g_current_dir[sizeof(g_current_dir)-1] = '\0';  
 
        printf("Next:(%d,%d) Dir:%s\n", nx, ny, dir);  
        printf("Checking obstacles...\n");  
 
        bool blocked = detectObstacle(dir);  
        vTaskDelay(pdMS_TO_TICKS(50));  
 
        if (blocked) {  
            if (soundNavMode && checkArrival()) {  
                printf("\n[Nav] *** ARRIVED AT SOUND SOURCE ***\n");  
                printf("      Mag growth + obstacle confirm arrival.\n\n");  
                snprintf(statusMessage, sizeof(statusMessage),  
                         "Arrived at sound source! Press Chase to follow again.");  
                navigationActive = false;  
                soundNavMode     = false;  
                for(int i=0;i<8;i++){ setLED(true); vTaskDelay(pdMS_TO_TICKS(120)); setLED(false); vTaskDelay(pdMS_TO_TICKS(120)); }  
                setLED(true);  
                vTaskDelay(pdMS_TO_TICKS(100));  
                continue;  
            }  
 
            printf("!!! BLOCKED - REPLANNING !!!\n");  
            snprintf(statusMessage, sizeof(statusMessage), "Obstacle! Replanning...");  
            scanAndMapObstacles(grid, curX, curY);  
            printMapToSerial();  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        printf("MOVING %s\n", dir);  
        snprintf(statusMessage, sizeof(statusMessage), "Moving %s", dir);  
        driveMecanum(dir);  
        // Side-to-side (strafe) moves use a separate delay from fwd/back.
        // Both are settable live from the web UI via /set_move_delay.
        bool isSide = (strcmp(dir,"left")==0 || strcmp(dir,"right")==0);
        vTaskDelay(pdMS_TO_TICKS(isSide ? g_move_delay_side_ms : g_move_delay_fwd_ms));
        stopMotors();  
        vTaskDelay(pdMS_TO_TICKS(200));  
 
        curX = nx; curY = ny;  
        currentStep++;
        g_total_steps_traveled++;  // FIX 2: accumulate total distance
        
        // Track absolute physical displacement independent of grid expansions/shifts
        g_net_dx += ddx;
        g_net_dy += ddy;
        
        visited.push_back({curX, curY});  
 
        printf("Moved to:(%d,%d)  DistToGoal:%.0f\n\n",  
               curX, curY, heuristic(curX,curY,goalX,goalY));  
 
        scanAndMapObstacles(grid, curX, curY);  
        if (currentStep % 5 == 0) printMapToSerial();  
 
        if (soundNavMode && checkArrival()) {  
            printf("\n[Nav] *** ARRIVED AT SOUND SOURCE (post-move check) ***\n\n");  
            snprintf(statusMessage, sizeof(statusMessage),  
                     "Arrived at sound source! Press Chase to follow again.");  
            navigationActive = false;  
            soundNavMode     = false;  
            for(int i=0;i<8;i++){ setLED(true); vTaskDelay(pdMS_TO_TICKS(120)); setLED(false); vTaskDelay(pdMS_TO_TICKS(120)); }  
            setLED(true);  
            vTaskDelay(pdMS_TO_TICKS(100));  
            continue;  
        }  
 
        vTaskDelay(pdMS_TO_TICKS(100));  
    }  
}  
 
// =============================================================================
//  MAIN ENTRY POINT
// =============================================================================
extern "C" void app_main() {
    // Initialize NVS (required for WiFi Access Point)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("\n========================================\n");
    printf("       SOUND ROVER INITIALIZING\n");
    printf("========================================\n\n");

    // Initialize Shared Mutex
    g_sound_mutex = xSemaphoreCreateMutex();

    // Initialize Hardware
    initLED();
    initMotorGPIO();
    initMotorPWM();
    initUltrasonicPins();
    initUltrasonicISR();

    // Initialize DSP & Microphones
    precompute_dsp();
    init_microphone_i2s();

    // Start Network & UI
    setupAP();
    start_webserver();

    // Launch Tasks
    // Core 0: WiFi / HTTP Server (handled by ESP-IDF internally) + Sound Processing
    xTaskCreatePinnedToCore(sound_task, "sound_task", 8192, NULL, 5, NULL, 0);

    // Core 1: Navigation, Path Planning, & Motor Control
    xTaskCreatePinnedToCore(nav_task, "nav_task", 8192, NULL, 5, NULL, 1);

    printf("\nSystem initialized! Connect to 'Sound_Rover' WiFi and go to http://192.168.4.1\n");
}
