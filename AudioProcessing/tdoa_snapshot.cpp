/* =================================================================================================
 * UMA-8 TDOA Snapshot Sound Locator
 *
 * Description:
 * This program operates in a "one-shot" or "snapshot" mode, making it ideal for an
 * event-driven embedded system. It performs the following steps:
 * 1.  Calibrates the background noise floor automatically on startup.
 * 2.  Waits silently for a sound event that exceeds the dynamic noise threshold.
 * 3.  Captures a fixed duration of audio after the trigger.
 * 4.  Analyzes the capture to find the single loudest frame ("peak frame").
 * 5.  Calculates a high-precision angle and a rough distance estimate based on that frame.
 * 6.  Prints a single result and exits.
 *
 * =================================================================================================
 */

#define _USE_MATH_DEFINES
#include <cmath>
#define NOMINMAX
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "fft.hpp"

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <complex>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <mutex>

// --- Core Configuration ---
const int SAMPLE_RATE = 48000;
const int CHANNEL_COUNT = 8;
const float SPEED_OF_SOUND = 343.0f;
const float MIC_RADIUS = 0.045f;
const int FFT_SIZE = 1024;
const int HOP_SIZE = FFT_SIZE / 2;

// --- Detection & Capture Configuration ---
const float NOISE_THRESHOLD_MULTIPLIER = 5.0f; // Trigger when sound is 5x louder than background noise.
const int CALIBRATION_FRAMES = 40;            // Number of frames to measure for noise floor (~1 second).
const int CAPTURE_FRAMES = 300;            

// --- Bandpass Filter & Gain ---
const float MIN_FREQ = 300.0f;
const float MAX_FREQ = 3400.0f;
const float VOICE_FREQ_GAIN = 10.0f;

// --- Angle Calculation Parameters ---
const int ANGLE_SEARCH_STEP = 3;
const int NUM_ANGLES = 360 / ANGLE_SEARCH_STEP;

// --- DISTANCE ESTIMATION (NEEDS CALIBRATION!) ---
const float REFERENCE_DISTANCE = 1.0f;
const float REFERENCE_ENERGY = 0.05f;

// --- Type definitions ---
using Complex = std::complex<float>;
using ComplexVector = std::vector<Complex>;
using SteeringVector = std::vector<ComplexVector>;
using AudioFrame = std::vector<std::vector<float>>;

// --- Global Data Structures for Audio Callback ---
struct UserData {
    std::vector<float> audio_buffer;
    std::mutex buffer_mutex;
    size_t head = 0;
    size_t tail = 0;
};

const std::vector<std::pair<float, float>> MIC_POSITIONS = {
    {0.0f, 0.0f}, // Mic 0 (center)
    {MIC_RADIUS * cosf(0.0f * M_PI / 180.0f), MIC_RADIUS * sinf(0.0f * M_PI / 180.0f)},
    {MIC_RADIUS * cosf(60.0f * M_PI / 180.0f), MIC_RADIUS * sinf(60.0f * M_PI / 180.0f)},
    {MIC_RADIUS * cosf(120.0f * M_PI / 180.0f), MIC_RADIUS * sinf(120.0f * M_PI / 180.0f)},
    {MIC_RADIUS * cosf(180.0f * M_PI / 180.0f), MIC_RADIUS * sinf(180.0f * M_PI / 180.0f)},
    {MIC_RADIUS * cosf(240.0f * M_PI / 180.0f), MIC_RADIUS * sinf(240.0f * M_PI / 180.0f)},
    {MIC_RADIUS * cosf(300.0f * M_PI / 180.0f), MIC_RADIUS * sinf(300.0f * M_PI / 180.0f)},
    {0.0f, 0.0f}, // Mic 7 (spare)
};


// --- Forward Declarations for Helper Functions ---
std::vector<SteeringVector> precompute_steering_vectors();
float parabolic_interpolation(float y_minus, float y_zero, float y_plus, int peak_index);
std::pair<int, std::vector<float>> calculate_doa_powers(std::vector<ComplexVector>& channel_ffts, const std::vector<SteeringVector>& all_steering_vectors);
float calculate_rms_energy(const AudioFrame& frame);
float estimate_distance(float measured_energy);
void read_new_hop(UserData& userData, std::vector<float>& dest_buffer);


// This callback runs in a separate thread, continuously feeding audio into a circular buffer.
void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    (void)pOutput;
    auto* pUserData = static_cast<UserData*>(pDevice->pUserData);
    if (pUserData == nullptr) return;

    const float* pInputF32 = (const float*)pInput;
    std::lock_guard<std::mutex> lock(pUserData->buffer_mutex);

    for (ma_uint32 i = 0; i < frameCount * CHANNEL_COUNT; ++i) {
        pUserData->audio_buffer[pUserData->head] = pInputF32[i];
        pUserData->head = (pUserData->head + 1) % pUserData->audio_buffer.size();
    }
}


// =================================================================================================
//  Main Application Logic
// =================================================================================================
int main() {
    // --- 1. Initialization ---
    std::cout << "Initializing..." << std::endl;
    auto all_steering_vectors = precompute_steering_vectors();

    UserData userData;
    // Buffer for 4 seconds of audio to prevent overflow
    userData.audio_buffer.resize(SAMPLE_RATE * CHANNEL_COUNT * 4);

    ma_device_config deviceConfig = ma_device_config_init(ma_device_type_capture);
    deviceConfig.capture.format = ma_format_f32;
    deviceConfig.capture.channels = CHANNEL_COUNT;
    deviceConfig.sampleRate = SAMPLE_RATE;
    deviceConfig.periodSizeInFrames = HOP_SIZE;
    deviceConfig.dataCallback = data_callback;
    deviceConfig.pUserData = &userData;

    ma_device device;
    if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
        std::cerr << "Failed to initialize capture device." << std::endl;
        return -1;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        std::cerr << "Failed to start device." << std::endl;
        ma_device_uninit(&device);
        return -1;
    }

    std::vector<float> hop_buffer(HOP_SIZE * CHANNEL_COUNT);
    std::vector<float> window(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
    }

    // --- 2. Noise Floor Calibration ---
    std::cout << "Calibrating background noise... Please be quiet." << std::endl;
    float noise_floor_sum = 0.0f;
    std::vector<float> full_frame_buffer(FFT_SIZE * CHANNEL_COUNT, 0.0f);

    for (int i = 0; i < CALIBRATION_FRAMES; ++i) {
        read_new_hop(userData, hop_buffer);
        memmove(full_frame_buffer.data(), full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        memcpy(full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, hop_buffer.data(), HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        
        AudioFrame current_frame(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
        for(int s = 0; s < FFT_SIZE; ++s) for(int ch = 0; ch < CHANNEL_COUNT; ++ch) current_frame[ch][s] = full_frame_buffer[s * CHANNEL_COUNT + ch];
        noise_floor_sum += calculate_rms_energy(current_frame);
    }
    float noise_floor_avg = noise_floor_sum / CALIBRATION_FRAMES;
    float dynamic_threshold = noise_floor_avg * NOISE_THRESHOLD_MULTIPLIER;
    std::cout << "Calibration complete. Noise Floor RMS: " << noise_floor_avg << ", Detection Threshold: " << dynamic_threshold << std::endl;

    // --- 3. Wait for Trigger ---
    std::cout << "Listening for sound event..." << std::endl;
    while (true) {
        read_new_hop(userData, hop_buffer);
        memmove(full_frame_buffer.data(), full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        memcpy(full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, hop_buffer.data(), HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        
        AudioFrame current_frame(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
        for(int s = 0; s < FFT_SIZE; ++s) for(int ch = 0; ch < CHANNEL_COUNT; ++ch) current_frame[ch][s] = full_frame_buffer[s * CHANNEL_COUNT + ch];
        
        if (calculate_rms_energy(current_frame) > dynamic_threshold) {
            std::cout << "Sound event detected! Capturing..." << std::endl;
            break;
        }
    }

    // --- 4. Capture Event Data ---
    std::vector<AudioFrame> captured_frames;
    captured_frames.reserve(CAPTURE_FRAMES);
    for (int i = 0; i < CAPTURE_FRAMES; ++i) {
        read_new_hop(userData, hop_buffer);
        memmove(full_frame_buffer.data(), full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        memcpy(full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, hop_buffer.data(), HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        
        AudioFrame current_frame(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
        for(int s = 0; s < FFT_SIZE; ++s) for(int ch = 0; ch < CHANNEL_COUNT; ++ch) current_frame[ch][s] = full_frame_buffer[s * CHANNEL_COUNT + ch];
        captured_frames.push_back(current_frame);
    }
    ma_device_stop(&device);
    ma_device_uninit(&device);
    std::cout << "Capture complete. Processing..." << std::endl;

    // --- 5. Process Captured Data ---
    int peak_frame_index = -1;
    float max_energy = -1.0f;
    for (size_t i = 0; i < captured_frames.size(); ++i) {
        float energy = calculate_rms_energy(captured_frames[i]);
        if (energy > max_energy) {
            max_energy = energy;
            peak_frame_index = i;
        }
    }

    if (peak_frame_index == -1) {
        std::cerr << "Error: Could not find a peak frame in the capture." << std::endl;
        return -1;
    }
    
    AudioFrame& peak_frame = captured_frames[peak_frame_index];
    for (auto& channel : peak_frame) {
        for (int i = 0; i < FFT_SIZE; ++i) channel[i] *= window[i];
    }
    
    std::vector<ComplexVector> channel_ffts(CHANNEL_COUNT);
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        channel_ffts[i].assign(peak_frame[i].begin(), peak_frame[i].end());
        Fft::transform(channel_ffts[i]);
    }

    auto result = calculate_doa_powers(channel_ffts, all_steering_vectors);
    int peak_angle_index = result.first;
    float final_angle = -1.0f;
    float final_distance = -1.0f;

    if (peak_angle_index != -1) {
        auto angle_powers = result.second;
        int prev_index = (peak_angle_index == 0) ? NUM_ANGLES - 1 : peak_angle_index - 1;
        int next_index = (peak_angle_index == NUM_ANGLES - 1) ? 0 : peak_angle_index + 1;
        final_angle = parabolic_interpolation(angle_powers[prev_index], angle_powers[peak_angle_index], angle_powers[next_index], peak_angle_index);
        final_distance = estimate_distance(max_energy);
    }

    // --- 6. Output Final Result ---
    std::cout << "\n========================================" << std::endl;
    std::cout << "         ANALYSIS COMPLETE" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Peak Frame RMS:        " << std::fixed << std::setprecision(4) << max_energy << " (used for distance)" << std::endl;
    if (final_angle >= 0) {
        std::cout << "Detected Angle:        " << std::fixed << std::setprecision(1) << final_angle << " degrees" << std::endl;
        std::cout << "Estimated Distance:    ~" << std::fixed << std::setprecision(2) << final_distance << " meters" << std::endl;
    } else {
        std::cout << "Could not determine angle." << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}


// =================================================================================================
//  Helper Function Implementations
// =================================================================================================

// Blocks until a new hop of audio is available from the callback and copies it.
void read_new_hop(UserData& userData, std::vector<float>& dest_buffer) {
    while (true) {
        size_t captured_head;
        {
            std::lock_guard<std::mutex> lock(userData.buffer_mutex);
            captured_head = userData.head;
        }

        if ((captured_head - userData.tail + userData.audio_buffer.size()) % userData.audio_buffer.size() >= HOP_SIZE * CHANNEL_COUNT) {
            std::lock_guard<std::mutex> lock(userData.buffer_mutex);
            for (size_t i = 0; i < HOP_SIZE * CHANNEL_COUNT; ++i) {
                dest_buffer[i] = userData.audio_buffer[(userData.tail + i) % userData.audio_buffer.size()];
            }
            userData.tail = (userData.tail + HOP_SIZE * CHANNEL_COUNT) % userData.audio_buffer.size();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

float calculate_rms_energy(const AudioFrame& frame) {
    float sum_sq = 0.0f;
    for (float sample : frame[0]) {
        sum_sq += sample * sample;
    }
    return std::sqrt(sum_sq / frame[0].size());
}

float estimate_distance(float measured_energy) {
    if (measured_energy < 1e-9) return -1.0f;
    return REFERENCE_DISTANCE * std::sqrt(REFERENCE_ENERGY / measured_energy);
}

std::vector<SteeringVector> precompute_steering_vectors() {
    std::vector<SteeringVector> all_steering_vectors(NUM_ANGLES);
    for (int i = 0; i < NUM_ANGLES; ++i) {
        int angle = i * ANGLE_SEARCH_STEP;
        all_steering_vectors[i].resize(CHANNEL_COUNT);
        float angle_rad = angle * M_PI / 180.0f;
        for (int mic = 1; mic <= 6; ++mic) {
            all_steering_vectors[i][mic].resize(FFT_SIZE / 2 + 1);
            float mic_x = MIC_POSITIONS[mic].first;
            float mic_y = MIC_POSITIONS[mic].second;
            float projection = mic_x * cosf(angle_rad) + mic_y * sinf(angle_rad);
            float time_delay = projection / SPEED_OF_SOUND;
            for (int k = 0; k <= FFT_SIZE / 2; ++k) {
                float freq = (float)k * SAMPLE_RATE / FFT_SIZE;
                float omega = 2.0f * M_PI * freq;
                all_steering_vectors[i][mic][k] = std::exp(Complex(0.0f, 1.0f) * omega * time_delay);
            }
        }
    }
    return all_steering_vectors;
}

float parabolic_interpolation(float y_minus, float y_zero, float y_plus, int peak_index) {
    float numerator = 0.5f * (y_minus - y_plus);
    float denominator = y_minus - (2.0f * y_zero) + y_plus;
    if (std::abs(denominator) < 1e-6) {
        return static_cast<float>(peak_index * ANGLE_SEARCH_STEP);
    }
    float p = numerator / denominator;
    float interpolated_angle = (static_cast<float>(peak_index) + p) * static_cast<float>(ANGLE_SEARCH_STEP);
    return fmod(fmod(interpolated_angle, 360.0f) + 360.0f, 360.0f);
}

std::pair<int, std::vector<float>> calculate_doa_powers(std::vector<ComplexVector>& channel_ffts, const std::vector<SteeringVector>& all_steering_vectors) {
    std::vector<float> angle_powers(NUM_ANGLES, 0.0f);
    int best_angle_index = -1;
    float max_power = -1.0f;
    const int min_bin = static_cast<int>(MIN_FREQ * FFT_SIZE / SAMPLE_RATE);
    const int max_bin = static_cast<int>(MAX_FREQ * FFT_SIZE / SAMPLE_RATE);
    for (auto& fft_vec : channel_ffts) {
        for (size_t k = 0; k < fft_vec.size(); ++k) {
            fft_vec[k] = (k >= min_bin && k <= max_bin) ? (fft_vec[k] * VOICE_FREQ_GAIN) : Complex(0.0f, 0.0f);
        }
    }
    for (int i = 0; i < NUM_ANGLES; ++i) {
        ComplexVector summed_spectrum(FFT_SIZE / 2 + 1, {0.0f, 0.0f});
        for (int mic = 1; mic <= 6; ++mic) {
            for (int k = min_bin; k <= max_bin; ++k) {
                summed_spectrum[k] += channel_ffts[mic][k] * std::conj(all_steering_vectors[i][mic][k]);
            }
        }
        float current_power = 0.0f;
        for (int k = min_bin; k <= max_bin; ++k) {
            current_power += std::norm(summed_spectrum[k]);
        }
        angle_powers[i] = current_power;
        if (current_power > max_power) {
            max_power = current_power;
            best_angle_index = i;
        }
    }
    return {best_angle_index, angle_powers};
}

