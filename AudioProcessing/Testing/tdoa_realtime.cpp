#define _USE_MATH_DEFINES
#include <cmath>

// By defining NOMINMAX, we prevent windows.h (often included by other libraries)
// from defining min() and max() as macros, which would conflict with std::min/max.
#define NOMINMAX

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "fft.hpp" // Assumes this library works with std::complex<float>
#include <fstream>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <string>
#include <complex>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <cstdlib>

// --- Core Configuration ---
const int SAMPLE_RATE = 48000;
const int CHANNEL_COUNT = 8;
const float SPEED_OF_SOUND = 343.0f;
const float MIC_RADIUS = 0.045f; // 45mm for UMA-8

// --- TDOA Processing Configuration ---
const int FFT_SIZE = 1024;
const int HOP_SIZE = FFT_SIZE / 2;
const float ENERGY_THRESHOLD = 0.0005f;
const float VOICE_FREQ_GAIN = 10000.0f; // Amplification for voice frequencies

// --- Bandpass Filter Configuration ---
const float MIN_FREQ = 300.0f;
const float MAX_FREQ = 600.0f;

// --- OPTIMIZATION & ACCURACY PARAMETERS ---
const int ANGLE_SEARCH_STEP = 3; // Search every 3 degrees for speed.
const int NUM_ANGLES = 360 / ANGLE_SEARCH_STEP;
const int CONFIRMATION_THRESHOLD = 4; // Angle must be stable for this many frames.

// --- Type definitions for clarity (USING FLOAT FOR PERFORMANCE) ---
using Complex = std::complex<float>;
using ComplexVector = std::vector<Complex>;
using SteeringVector = std::vector<ComplexVector>;

// --- Global Data Structures ---
struct UserData {
    std::vector<float> audio_buffer;
    std::mutex buffer_mutex;
    size_t head = 0;
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


//Pre-computes the phase shifts for our coarse angle search
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

//Estimates the precise angle by fitting a parabola to the three peak power points.
float parabolic_interpolation(float y_minus, float y_zero, float y_plus, int peak_index) {
    //Formula for the vertex of a parabola given three equally spaced points
    float numerator = 0.5f * (y_minus - y_plus);
    float denominator = y_minus - (2.0f * y_zero) + y_plus;
    if (std::abs(denominator) < 1e-6) { // Avoid division by zero
        // If the denominator is zero, the points are colinear, so no peak exists. Return the coarse index.
        return static_cast<float>(peak_index * ANGLE_SEARCH_STEP);
    }
    float p = numerator / denominator;
    // The interpolated angle is the coarse angle plus the offset 'p' scaled by the step size.
    return (static_cast<float>(peak_index) + p) * static_cast<float>(ANGLE_SEARCH_STEP);
}


// Frequency-Domain Beamforming with Coarse Search and Power Value Return
std::pair<int, std::vector<float>> calculate_doa_powers(
    std::vector<ComplexVector>& channel_ffts,
    const std::vector<SteeringVector>& all_steering_vectors) {

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

void print_debug_dashboard(float rms_energy, float final_angle) {
    #ifdef _WIN32
        system("cls");
    #else
        std::cout << "\033[2J\033[H";
    #endif
    
    std::cout << "===== UMA-8 TDOA Real-Time Debug Dashboard (Optimized) =====\n";
    std::cout << "Listening for human voice (" << MIN_FREQ << "-" << MAX_FREQ << " Hz)...\n";
    std::cout << "------------------------------------------------\n";
    std::cout << "RMS Energy: " << std::fixed << std::setprecision(4) << rms_energy 
              << " (Threshold: " << ENERGY_THRESHOLD << ")" << (rms_energy >= ENERGY_THRESHOLD ? " [SOUND DETECTED]" : " [SILENT]")
              << "       \n";
    std::cout << "------------------------------------------------\n";

    if (final_angle >= 0) {
        std::cout << "Final Estimated Angle: " << std::fixed << std::setprecision(1) << final_angle << " degrees\n";
    } else {
        std::cout << "Final Estimated Angle: N/A\n";
    }

    std::string compass_line(45, ' ');
    if (final_angle >= 0) {
        int pos = static_cast<int>(roundf((final_angle / 360.0f) * 44.0f));
        pos = std::max(0, std::min(44, pos));
        compass_line[pos] = 'V';
    }
    std::cout << "\n 0" << std::string(20, '-') << "180" << std::string(20, '-') << "359\n";
    std::cout << "[" << compass_line << "]\n";
    std::cout << "\nPress Enter to quit.\n" << std::flush;
}

void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    (void)pOutput;
    UserData* pUserData = (UserData*)pDevice->pUserData;
    const float* pInputF32 = (const float*)pInput;
    
    std::lock_guard<std::mutex> lock(pUserData->buffer_mutex);
    for (ma_uint32 i = 0; i < frameCount * CHANNEL_COUNT; ++i) {
        pUserData->audio_buffer[pUserData->head] = pInputF32[i];
        pUserData->head = (pUserData->head + 1) % pUserData->audio_buffer.size();
    }
}

//MAIN

int main() {
    std::cout << "Pre-computing steering vectors for " << NUM_ANGLES << " angles..." << std::endl;
    auto all_steering_vectors = precompute_steering_vectors();
    std::cout << "Done." << std::endl;

    UserData userData;
    userData.audio_buffer.resize(SAMPLE_RATE * CHANNEL_COUNT * 2);

    ma_device_config deviceConfig = ma_device_config_init(ma_device_type_capture);
    deviceConfig.capture.format   = ma_format_f32;
    deviceConfig.capture.channels = CHANNEL_COUNT;
    deviceConfig.sampleRate       = SAMPLE_RATE;
    deviceConfig.dataCallback     = data_callback;
    deviceConfig.pUserData        = &userData;
    deviceConfig.periodSizeInFrames = HOP_SIZE;

    ma_device device;
    if (ma_device_init(NULL, &deviceConfig, &device) != MA_SUCCESS) {
        std::cerr << "Failed to initialize capture device." << std::endl;
        return -1;
    }
    ma_device_start(&device);

    size_t processing_head = 0;
    std::vector<float> process_buffer(FFT_SIZE * CHANNEL_COUNT);
    std::vector<float> window(FFT_SIZE);
    for(int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
    }

    // --- State variables for temporal smoothing ---
    float stable_angle = -1.0f;
    float potential_angle = -1.0f;
    int confirmation_counter = 0;

    while (true) {
        if (std::cin.rdbuf()->in_avail() > 0) break;

        size_t captured_head;
        {
            std::lock_guard<std::mutex> lock(userData.buffer_mutex);
            captured_head = userData.head;
        }
        
        if ((captured_head - processing_head + userData.audio_buffer.size()) % userData.audio_buffer.size() >= HOP_SIZE * CHANNEL_COUNT) {
            {
                std::lock_guard<std::mutex> lock(userData.buffer_mutex);
                size_t start_pos = (processing_head - (FFT_SIZE / 2 * CHANNEL_COUNT) + userData.audio_buffer.size()) % userData.audio_buffer.size();
                for (int i = 0; i < FFT_SIZE * CHANNEL_COUNT; ++i) {
                   process_buffer[i] = userData.audio_buffer[(start_pos + i) % userData.audio_buffer.size()];
                }
            }
            processing_head = (processing_head + HOP_SIZE * CHANNEL_COUNT) % userData.audio_buffer.size();
            
            std::vector<std::vector<float>> channels(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
            for(int i = 0; i < FFT_SIZE; ++i) {
                for(int j = 0; j < CHANNEL_COUNT; ++j) {
                    channels[j][i] = process_buffer[i * CHANNEL_COUNT + j] * window[i];
                }
            }

            float rms_energy = 0.0f;
            for (float sample : channels[0]) rms_energy += sample * sample;
            rms_energy = std::sqrt(rms_energy / channels[0].size());
            
            if (rms_energy >= ENERGY_THRESHOLD) {
                std::vector<ComplexVector> channel_ffts(CHANNEL_COUNT);
                for (int i = 0; i < CHANNEL_COUNT; ++i) {
                    channel_ffts[i].assign(channels[i].begin(), channels[i].end());
                    Fft::transform(channel_ffts[i]); // Assumes your fft lib has this function
                }

                auto result = calculate_doa_powers(channel_ffts, all_steering_vectors);
                int peak_index = result.first;
                
                if (peak_index != -1) {
                    auto angle_powers = result.second;
                    // Get neighbors for interpolation, handling wraparound at 0/360 degrees
                    int prev_index = (peak_index == 0) ? NUM_ANGLES - 1 : peak_index - 1;
                    int next_index = (peak_index == NUM_ANGLES - 1) ? 0 : peak_index + 1;

                    float interpolated_angle = parabolic_interpolation(
                        angle_powers[prev_index], angle_powers[peak_index], angle_powers[next_index], peak_index
                    );

                    // --- Temporal Smoothing Logic ---
                    if (std::abs(interpolated_angle - potential_angle) < 10.0f) { // Check if new angle is close to the last one
                        confirmation_counter++;
                    } else {
                        potential_angle = interpolated_angle;
                        confirmation_counter = 1;
                    }
                    if (confirmation_counter >= CONFIRMATION_THRESHOLD) {
                        stable_angle = potential_angle;
                    }
                }
            } else {
                // If silent, reset the smoothing state
                potential_angle = -1.0f;
                stable_angle = -1.0f;
                confirmation_counter = 0;
            }
            
            print_debug_dashboard(rms_energy, stable_angle);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ma_device_uninit(&device);
    return 0;
}