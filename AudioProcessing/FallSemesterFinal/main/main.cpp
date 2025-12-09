#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <complex>
#include <algorithm>
#include <set>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// --- WIFI CONFIGURATION ---
#define WIFI_SSID      "SamiPhone"  
#define WIFI_PASS      "password" 

// --- SERVER CONFIGURATION ---
#define SERVER_PORT     9003
#define STATUS_LED_GPIO GPIO_NUM_2 

// --- PIN CONFIGURATION ---
#define I2S_BCK_IO      GPIO_NUM_42
#define I2S_WS_IO       GPIO_NUM_41
#define I2S_DO_0_IO     GPIO_NUM_16 // I2S0 Data In
#define I2S_DO_1_IO     GPIO_NUM_40 // I2S1 Data In

// --- AUDIO CONFIGURATION ---
#define SAMPLE_RATE     16000
#define FFT_SIZE        1024        
#define BUFF_SIZE       FFT_SIZE
#define SPEED_OF_SOUND  343.0f

// --- ALGORITHM TUNING ---
#define TARGET_FREQ     450.0f      
// Tighter Sigma = Stricter Filter. 5.0f means it ignores 430hz or 470hz.
#define TARGET_SIGMA    5.0f        
#define SMOOTHING       0.85f       
#define MAG_THRESHOLD   200000.0f    
#define ANGLE_STEP      2           // Scans every 2 degrees for higher precision

// --- DISTANCE ESTIMATION CONSTANTS ---
const float REFERENCE_ENERGY = 0.05f;
const float REFERENCE_DISTANCE = 0.05f;

// --- MIC GEOMETRY ---
// 0,0 is the center of the square.
// +Y is "North" (Top), +X is "East" (Right)
struct Point { float x, y; };

const Point MIC_POSITIONS[4] = {
    // Mic 0: Bottom Right (+X, -Y)
    { -0.0381f, -0.0254f}, 
    // Mic 1: Bottom Left  (-X, -Y)
    {-0.0381f, 0.0254f}, 
    // Mic 2: Top Left     (-X, +Y)
    {0.0381f,  -0.0254f}, 
    // Mic 3: Top Right    (+X, +Y)
    { 0.0381f,  0.0254f}  
};

// --- GLOBALS ---
static i2s_chan_handle_t rx_handle_0 = NULL;
static i2s_chan_handle_t rx_handle_1 = NULL;
static const char *TAG = "TDOA_FIX";

const float PI = 3.14159265358979323846f;
using Complex = std::complex<float>;

// DSP Globals
std::vector<std::vector<Complex>> steering_vectors; 
std::vector<Complex> smoothed_bins(4, Complex(0,0));
std::vector<float> gaussian_weights; //used to filter the 450Hz signal

// WebServer Globals
static httpd_handle_t server = NULL;
std::set<int> g_client_sockets;
std::mutex g_client_mutex;
volatile bool g_wifi_connected = false;

float g_last_valid_angle = 0.0f; // Stores last valid angle for persistence

// --- WIFI HANDLERS ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        gpio_set_level(STATUS_LED_GPIO, 0);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Use this IP in your browser: http://" IPSTR ":9003", IP2STR(&event->ip_info.ip));
        g_wifi_connected = true;
        gpio_set_level(STATUS_LED_GPIO, 1);
    }
}

void wifi_init_sta(void) {
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

// --- WEBSOCKET HANDLERS ---
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int sockfd = httpd_req_to_sockfd(req);
        std::lock_guard<std::mutex> lock(g_client_mutex);
        g_client_sockets.insert(sockfd);
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        int sockfd = httpd_req_to_sockfd(req);
        std::lock_guard<std::mutex> lock(g_client_mutex);
        g_client_sockets.erase(sockfd);
        return ESP_OK;
    }
    return ESP_OK;
}

static const httpd_uri_t ws_route = {
    .uri        = "/",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = SERVER_PORT;
    config.ctrl_port = SERVER_PORT + 1; 
    config.max_open_sockets = 5; 
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &ws_route);
    }
}

void broadcast_data(float angle, float amplitude, float distance, 
                   const std::vector<float>& waveform, 
                   const std::vector<float>& spectrum) { //json output
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "doa", angle);
    cJSON_AddNumberToObject(root, "amplitude", amplitude);
    cJSON_AddNumberToObject(root, "distance", distance); 

    cJSON *waveArr = cJSON_CreateArray();
    int wave_skip = 4; 
    for(size_t i=0; i<waveform.size(); i+=wave_skip) cJSON_AddItemToArray(waveArr, cJSON_CreateNumber(waveform[i]));
    cJSON_AddItemToObject(root, "waveform", waveArr);

    cJSON *freqArr = cJSON_CreateArray();
    for(float val : spectrum) cJSON_AddItemToArray(freqArr, cJSON_CreateNumber(val));
    cJSON_AddItemToObject(root, "freq_bins", freqArr);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)json_str;
        ws_pkt.len = strlen(json_str);
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        std::lock_guard<std::mutex> lock(g_client_mutex);
        for (auto it = g_client_sockets.begin(); it != g_client_sockets.end(); ) {
            int fd = *it;
            esp_err_t err = httpd_ws_send_frame_async(server, fd, &ws_pkt);
            if (err != ESP_OK) it = g_client_sockets.erase(it);
            else ++it;
        }
        free(json_str);
    }
    cJSON_Delete(root);
}

//Math
void apply_hann_window(std::vector<float> &data) {
    for (size_t i = 0; i < data.size(); ++i) {
        float multiplier = 0.5f * (1.0f - cosf(2.0f * PI * i / (data.size() - 1)));
        data[i] *= multiplier;
    }
}

float calculate_rms_energy(const std::vector<float>& frame) { //Loudness of sound
    float sum_sq = 0.0f;
    if (frame.empty()) return 0.0f;
    for (float sample : frame) {
        float norm = sample / 8388608.0f; 
        sum_sq += norm * norm;
    }
    return std::sqrt(sum_sq / frame.size());
}

float estimate_distance(float measured_energy) { //uses invserse square law to calculate distance
    if (measured_energy < 1e-9) return -1.0f;
    return REFERENCE_DISTANCE * std::sqrt(REFERENCE_ENERGY / measured_energy);
}

// Parabolic Interpolation for Sub-Degree Accuracy
float interpolate_peak(float y_minus, float y_center, float y_plus, int center_angle) { //interpolates between microphones
    float denom = y_minus - 2.0f * y_center + y_plus;
    if (denom == 0.0f) return (float)center_angle;
    
    float delta = (y_minus - y_plus) / (2.0f * denom);
    return (float)center_angle + (delta * ANGLE_STEP);
}

void fft(std::vector<Complex> &x) { //Fast fourier transforms
    const size_t N = x.size();
    if (N <= 1) return;
    std::vector<Complex> odd;
    odd.reserve(N / 2);
    for (size_t i = 1; i < N; i += 2) odd.push_back(x[i]);
    std::vector<Complex> even;
    even.reserve(N / 2);
    for (size_t i = 0; i < N; i += 2) even.push_back(x[i]);
    fft(odd);
    fft(even);
    for (size_t k = 0; k < N / 2; ++k) {
        Complex t = std::polar(1.0f, -2.0f * PI * k / N) * odd[k];
        x[k] = even[k] + t;
        x[k + N / 2] = even[k] - t;
    }
}

// --- PRECOMPUTE DSP (Coordinate Fix Here) ---
void precompute_dsp() {
    steering_vectors.resize(360 / ANGLE_STEP);
    float omega = 2.0f * PI * TARGET_FREQ;

    // 1. Steering Vectors
    for (int i = 0; i < (360 / ANGLE_STEP); ++i) {
        int angle = i * ANGLE_STEP;
        steering_vectors[i].resize(4);
        float angle_rad = angle * PI / 180.0f;
        
        // --- KEY FIX: NAVIGATION COORDINATES ---
        // 0 deg = North (+Y), 90 deg = East (+X)
        // Standard Math: x = cos, y = sin.
        // Navigation:    x = sin, y = cos.
        float k_x = sinf(angle_rad); 
        float k_y = cosf(angle_rad);

        for (int mic = 0; mic < 4; ++mic) {
            float dist_proj = MIC_POSITIONS[mic].x * k_x + MIC_POSITIONS[mic].y * k_y;
            float delay = dist_proj / SPEED_OF_SOUND;
            // Negative phase aligns signal to center
            steering_vectors[i][mic] = std::polar(1.0f, -omega * delay);
        }
    }

    // 2. Gaussian Weights (Strict 450Hz Focus)
    gaussian_weights.resize(FFT_SIZE / 2, 0.0f);
    for(int k = 0; k < FFT_SIZE/2; k++) {
        float freq = (float)k * SAMPLE_RATE / FFT_SIZE;
        float diff = freq - TARGET_FREQ;
        // Gaussian Function: exp(-(x-u)^2 / (2*sigma^2))
        gaussian_weights[k] = expf(-(diff * diff) / (2.0f * TARGET_SIGMA * TARGET_SIGMA));
    }
    ESP_LOGI(TAG, "DSP Precomputed. Target: %.0fHz, Sigma: %.0fHz", TARGET_FREQ, TARGET_SIGMA);
}

// --- I2S INIT ---
void init_microphone_i2s() { //generates blck and word select, Channel 1 is slave
    i2s_chan_config_t chan_cfg_0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg_0.auto_clear = true; 
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_0, NULL, &rx_handle_0));

    i2s_std_config_t std_cfg_0 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DO_0_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_0, &std_cfg_0));

    i2s_chan_config_t chan_cfg_1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE); 
    chan_cfg_1.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_1, NULL, &rx_handle_1));

    i2s_std_config_t std_cfg_1 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO, 
            .ws = I2S_WS_IO,    
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DO_1_IO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_1, &std_cfg_1));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_0));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_1));
}

// --- MAIN LOOP ---
extern "C" void app_main(void) {
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_GPIO, 0);

    init_microphone_i2s();
    precompute_dsp();
    wifi_init_sta();
    start_webserver();

    // Scan Range for 450Hz (+/- 3 Sigma covers 99% of bell curve)
    float scan_bw = TARGET_SIGMA * 3.0f; 
    int start_bin = (int)((TARGET_FREQ - scan_bw) * FFT_SIZE / SAMPLE_RATE); //filter to bin at 45z
    int end_bin = (int)((TARGET_FREQ + scan_bw) * FFT_SIZE / SAMPLE_RATE);
    if(start_bin < 1) start_bin = 1;
    if(end_bin >= FFT_SIZE/2) end_bin = FFT_SIZE/2 - 1;

    int32_t *raw_buf_0 = (int32_t *)calloc(BUFF_SIZE * 2, sizeof(int32_t));
    int32_t *raw_buf_1 = (int32_t *)calloc(BUFF_SIZE * 2, sizeof(int32_t));
    size_t bytes_read = 0;

    std::vector<std::vector<float>> time_domain(4, std::vector<float>(FFT_SIZE));
    std::vector<std::vector<Complex>> freq_domain(4, std::vector<Complex>(FFT_SIZE));

    int loop_counter = 0;

    while (1) {
        if (!g_wifi_connected) {
            if (loop_counter % 12 == 0) {
                int level = gpio_get_level(STATUS_LED_GPIO);
                gpio_set_level(STATUS_LED_GPIO, !level);
            }
        }
        loop_counter++;

        esp_err_t res1 = i2s_channel_read(rx_handle_1, raw_buf_1, BUFF_SIZE * 2 * 4, &bytes_read, 100);
        esp_err_t res0 = i2s_channel_read(rx_handle_0, raw_buf_0, BUFF_SIZE * 2 * 4, &bytes_read, 100);

        if (res0 == ESP_OK && res1 == ESP_OK) {
            
            // De-interleave
            for (int i = 0; i < BUFF_SIZE; i++) {
                time_domain[0][i] = (float)(raw_buf_0[i*2] >> 8);
                time_domain[1][i] = (float)(raw_buf_0[i*2+1] >> 8);
                time_domain[2][i] = (float)(raw_buf_1[i*2] >> 8);
                time_domain[3][i] = (float)(raw_buf_1[i*2+1] >> 8);
            }

            // ADDED: Calculate and Print RMS for each Mic
            float rms[4];
            for (int m = 0; m < 4; m++) {
                rms[m] = calculate_rms_energy(time_domain[m]);
            }

            // FFT
            for (int m = 0; m < 4; m++) {
                apply_hann_window(time_domain[m]);
                for(int k=0; k<FFT_SIZE; k++) freq_domain[m][k] = Complex(time_domain[m][k], 0);
                fft(freq_domain[m]);
            }

            float max_mag = 0;
            Complex weighted_phasors[4] = {0,0,0,0};

            // --- FILTERING & PHASOR EXTRACTION ---
            for (int k = start_bin; k <= end_bin; k++) {
                float weight = gaussian_weights[k];
                if(weight < 0.01f) continue;

                for(int m=0; m<4; m++) {
                    float mag = std::abs(freq_domain[m][k]);
                    if(mag > max_mag) max_mag = mag;

                    // SRP-PHAT: Normalize magnitude to 1.0 (Unit Vector)
                    // Then apply Gaussian Weight
                    if (mag > 0.0001f) {
                        Complex phat = freq_domain[m][k] / mag; 
                        weighted_phasors[m] += phat * weight;
                    }
                }
            }

            float final_angle = g_last_valid_angle; 
            float display_amp = 0.0f;
            float display_dist = -1.0f;

            if (max_mag > MAG_THRESHOLD) {
                // Temporal Smoothing
                for(int m=0; m<4; m++) {
                    smoothed_bins[m] = (SMOOTHING * smoothed_bins[m]) + ((1.0f - SMOOTHING) * weighted_phasors[m]);
                }

                // Beamforming
                float best_energy = -1.0f;
                int best_idx = 0;
                int num_angles = 360 / ANGLE_STEP;
                std::vector<float> energies(num_angles);

                for (int i = 0; i < num_angles; ++i) {
                    Complex beam_sum(0, 0);
                    for (int m = 0; m < 4; m++) {
                        beam_sum += smoothed_bins[m] * steering_vectors[i][m];
                    }
                    float e = std::norm(beam_sum);
                    energies[i] = e;
                    if (e > best_energy) {
                        best_energy = e;
                        best_idx = i;
                    }
                }

                // Parabolic Interpolation for Precision
                int prev_idx = (best_idx == 0) ? num_angles - 1 : best_idx - 1;
                int next_idx = (best_idx == num_angles - 1) ? 0 : best_idx + 1;
                final_angle = interpolate_peak(energies[prev_idx], energies[best_idx], energies[next_idx], best_idx * ANGLE_STEP);
                
                // Wrap angle to 0-360
                if(final_angle < 0) final_angle += 360.0f;
                if(final_angle >= 360.0f) final_angle -= 360.0f;

                g_last_valid_angle = final_angle;

                display_amp = std::min(1.0f, max_mag / (MAG_THRESHOLD * 5.0f));
                // Use Mic 0 for distance estimation, could use average of all 4
                display_dist = estimate_distance(rms[0]); 

                // ADDED: Print Output to Terminal
                printf("M0:%.4f M1:%.4f M2:%.4f M3:%.4f | Angle: %.1f deg\n", 
                       rms[0], rms[1], rms[2], rms[3], final_angle);

            } else {
                for(int m=0; m<4; m++) smoothed_bins[m] *= 0.9f;
                // Silent state print
                printf("M0:%.4f M1:%.4f M2:%.4f M3:%.4f | (Quiet)\n", 
                       rms[0], rms[1], rms[2], rms[3]);
            }

            if (!g_client_sockets.empty()) {
                // Just send a small spectrum slice for UI
                std::vector<float> spec_slice;
                int center = (int)(TARGET_FREQ * FFT_SIZE / SAMPLE_RATE);
                for(int k=center-30; k<center+30; k++) {
                     if(k>0 && k<FFT_SIZE/2) spec_slice.push_back(std::abs(freq_domain[0][k]));
                }
                broadcast_data(final_angle, display_amp, display_dist, time_domain[0], spec_slice); //outputs information
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    free(raw_buf_0);
    free(raw_buf_1);
}