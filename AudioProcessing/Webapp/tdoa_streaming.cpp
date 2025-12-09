// --- START OF COMPILATION FIXES ---
#define _WIN32_WINNT 0x0A00
#define _USE_MATH_DEFINES
#define NOMINMAX
// --- END OF COMPILATION FIXES ---

// --- PRIMARY HEADERS ---
#include <ixwebsocket/ixwebsocket/IXWebSocketServer.h>
#include <ixwebsocket/ixwebsocket/IXNetSystem.h>

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
// --- END PRIMARY HEADERS ---

/* =================================================================================================
 * UMA-8 TDOA STREAMING Sound Locator + Distance Estimation
 * =================================================================================================
 */

// Includes
#include <cmath>
#include "fft.hpp"
#include "json.hpp" // For nlohmann/json

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <complex>
#include <algorithm>
#include <mutex>
#include <set>
#include <atomic>
#include <csignal>
#include <numeric> 

// --- WebSocket Server Globals ---
std::shared_ptr<ix::WebSocketServer> g_ws_server;
std::set<std::shared_ptr<ix::WebSocket>> g_connections;
std::mutex g_connection_mutex;
std::atomic<bool> g_running = true;

using json = nlohmann::json;

// --- Core Configuration ---
const int SAMPLE_RATE = 48000;
const int CHANNEL_COUNT = 8;
const float SPEED_OF_SOUND = 343.0f;
const float MIC_RADIUS = 0.045f;
const int FFT_SIZE = 1024;
const int HOP_SIZE = FFT_SIZE / 2;
const int SERVER_PORT = 9003; // Ensure this matches index.html
const long long LOOP_SLEEP_MS = 40;
const int ANGLE_SEARCH_STEP = 3;
const int NUM_ANGLES = 360 / ANGLE_SEARCH_STEP;

// --- Distance Estimation Constants (Matched to Snapshot Code) ---
const float REFERENCE_ENERGY = 0.05f;
const float REFERENCE_DISTANCE = 0.05f;

// --- NEW/UPDATED Configuration ---
const float NOISE_THRESHOLD_MULTIPLIER = 3.0f;  
const int CALIBRATION_FRAMES = 200;             
const float CALIBRATION_PERCENTILE = 0.5f;      

// --- NEW Target Frequency Configuration ---
const float TARGET_FREQ = 500.0f;               
const float FREQ_BANDWIDTH = 100.0f;            

// --- Derived Frequency Constants ---
const float MIN_FREQ = std::max(0.0f, TARGET_FREQ - FREQ_BANDWIDTH / 2.0f);
const float MAX_FREQ = std::min((float)SAMPLE_RATE / 2.0f, TARGET_FREQ + FREQ_BANDWIDTH / 2.0f);

// --- Type definitions ---
using Complex = std::complex<float>;
using ComplexVector = std::vector<Complex>;
using SteeringVector = std::vector<ComplexVector>;
using AudioFrame = std::vector<std::vector<float>>;

// --- Global Data Structures ---
struct UserData {
    std::vector<float> audio_buffer;
    std::mutex buffer_mutex;
    size_t head = 0;
    size_t tail = 0;
};

// --- Global for Robust Calibration ---
std::vector<float> g_noise_floor_spectrum;

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


// --- Forward Declarations ---
std::vector<SteeringVector> precompute_steering_vectors();
float parabolic_interpolation(float y_minus, float y_zero, float y_plus, int peak_index);
std::pair<int, std::vector<float>> calculate_doa_powers(
    std::vector<ComplexVector>& channel_ffts,
    const std::vector<SteeringVector>& all_steering_vectors,
    const std::vector<float>& noise_floor
);
float calculate_rms_energy(const AudioFrame& frame);
float estimate_distance(float measured_energy); // Added declaration
void read_new_hop(UserData& userData, std::vector<float>& dest_buffer);
std::vector<float> normalize_waveform(const std::vector<float>& wave);
std::vector<float> get_freq_bins(const ComplexVector& fft_data); 

// --- WebSocket Server Handlers ---

void broadcast_json(const json& msg) {
    std::lock_guard<std::mutex> lock(g_connection_mutex);
    std::string msg_str = msg.dump();
    auto clients = g_connections; 
    for (auto& session : clients) {
        session->send(msg_str);
    }
}

void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    (void)pOutput;
    auto* pUserData = static_cast<UserData*>(pDevice->pUserData);
    if (pUserData == nullptr || !g_running) return;

    const float* pInputF32 = (const float*)pInput;
    std::lock_guard<std::mutex> lock(pUserData->buffer_mutex);

    for (ma_uint32 i = 0; i < frameCount * CHANNEL_COUNT; ++i) {
        pUserData->audio_buffer[pUserData->head] = pInputF32[i];
        pUserData->head = (pUserData->head + 1) % pUserData->audio_buffer.size();
    }
}

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCaught SIGINT, shutting down..." << std::endl;
        g_running = false;
        if (g_ws_server) {
            g_ws_server->stop();
        }
    }
}

// =================================================================================================
//  Main Application Logic
// =================================================================================================
int main() {
    ix::initNetSystem();
    // --- 1. Initialization ---
    std::cout << "Initializing..." << std::endl;
    std::cout << "Target Frequency: " << TARGET_FREQ << " Hz" << std::endl;
    
    auto all_steering_vectors = precompute_steering_vectors();
    g_noise_floor_spectrum.resize(FFT_SIZE / 2 + 1, 0.0f);

    // --- 1a. Start WebSocket Server ---
    // IMPORTANT: Binding to "0.0.0.0" allows connections from other devices on the network
    g_ws_server = std::make_shared<ix::WebSocketServer>(SERVER_PORT, "0.0.0.0");
    
    g_ws_server->setOnConnectionCallback(
        [](std::weak_ptr<ix::WebSocket> weakSocket,
        std::shared_ptr<ix::ConnectionState> connectionState)
        {
            auto socket = weakSocket.lock();
            if (!socket) return;
            {
                std::lock_guard<std::mutex> lock(g_connection_mutex);
                g_connections.insert(socket);
            }
            std::cout << "Client connected! ID: " << connectionState->getId() << std::endl;

            socket->setOnMessageCallback(
                [socket, connectionState](const ix::WebSocketMessagePtr& msg)
                {
                    if (msg->type == ix::WebSocketMessageType::Close)
                    {
                        std::lock_guard<std::mutex> lock(g_connection_mutex);
                        g_connections.erase(socket);
                        std::cout << "Client disconnected. ID: " << connectionState->getId() << std::endl;
                    }
                }
            );
        }
    );

    std::thread ws_thread([]() {
        auto res = g_ws_server->listen();
        if (!res.first) {
            std::cerr << "FATAL: listen() failed! Error: " << res.second << std::endl;
            return;
        }
        g_ws_server->start();
        std::cout << "WebSocket server running at ws://0.0.0.0:" << SERVER_PORT << "/" << std::endl;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        g_ws_server->stop();
    });

    // --- 1b. Initialize Audio Device ---
    UserData userData;
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
        g_ws_server->stop();
        if(ws_thread.joinable()) ws_thread.join();
        return -1;
    }

    if (ma_device_start(&device) != MA_SUCCESS) {
        std::cerr << "Failed to start device." << std::endl;
        ma_device_uninit(&device);
        g_ws_server->stop();
        if(ws_thread.joinable()) ws_thread.join();
        return -1;
    }

    std::vector<float> hop_buffer(HOP_SIZE * CHANNEL_COUNT);
    std::vector<float> window(FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
    }
    
    signal(SIGINT, signal_handler);

    // --- 2. ROBUST Noise Floor Calibration ---
    std::cout << "Calibrating background noise... Please be quiet." << std::endl;
    
    std::vector<std::vector<float>> calibration_spectra(CALIBRATION_FRAMES, std::vector<float>(FFT_SIZE / 2 + 1));
    std::vector<float> full_frame_buffer(FFT_SIZE * CHANNEL_COUNT, 0.0f);
    AudioFrame current_frame(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
    ComplexVector fft_buffer(FFT_SIZE);

    for (int i = 0; i < CALIBRATION_FRAMES; ++i) {
        if (!g_running) break;
        read_new_hop(userData, hop_buffer);

        memmove(full_frame_buffer.data(), full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        memcpy(full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, hop_buffer.data(), HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        
        for(int s = 0; s < FFT_SIZE; ++s) {
            current_frame[0][s] = full_frame_buffer[s * CHANNEL_COUNT + 0];
            fft_buffer[s] = current_frame[0][s] * window[s];
        }

        Fft::transform(fft_buffer);

        for (size_t k = 0; k <= FFT_SIZE / 2; ++k) {
            calibration_spectra[i][k] = std::norm(fft_buffer[k]);
        }
    }

    if (g_running) {
        std::vector<float> bin_powers(CALIBRATION_FRAMES);
        for (size_t k = 0; k <= FFT_SIZE / 2; ++k) {
            for (int i = 0; i < CALIBRATION_FRAMES; ++i) {
                bin_powers[i] = calibration_spectra[i][k];
            }
            std::sort(bin_powers.begin(), bin_powers.end());
            int percentile_index = static_cast<int>(CALIBRATION_FRAMES * CALIBRATION_PERCENTILE);
            g_noise_floor_spectrum[k] = bin_powers[percentile_index];
        }
        std::cout << "Calibration complete." << std::endl;
    } else {
        ma_device_stop(&device);
        ma_device_uninit(&device);
        if(ws_thread.joinable()) ws_thread.join();
        return 0;
    }


    // --- 3. Continuous Processing Loop ---
    std::cout << "Listening and streaming data... (Press Ctrl+C to stop)" << std::endl;
    json data_packet;
    const int min_bin = static_cast<int>(MIN_FREQ * FFT_SIZE / SAMPLE_RATE);
    const int max_bin = static_cast<int>(MAX_FREQ * FFT_SIZE / SAMPLE_RATE);

    while (g_running) {
        read_new_hop(userData, hop_buffer);
        if (!g_running) break;
        
        memmove(full_frame_buffer.data(), full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        memcpy(full_frame_buffer.data() + HOP_SIZE * CHANNEL_COUNT, hop_buffer.data(), HOP_SIZE * CHANNEL_COUNT * sizeof(float));
        
        AudioFrame current_frame(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
        std::vector<ComplexVector> channel_ffts(CHANNEL_COUNT, ComplexVector(FFT_SIZE));

        for(int ch = 0; ch < CHANNEL_COUNT; ++ch) {
            for(int s = 0; s < FFT_SIZE; ++s) {
                current_frame[ch][s] = full_frame_buffer[s * CHANNEL_COUNT + ch];
                channel_ffts[ch][s] = current_frame[ch][s] * window[s];
            }
            Fft::transform(channel_ffts[ch]);
        }
        
        // --- Detection Logic ---
        float target_signal_energy = 0.0f;
        float target_noise_energy = 0.0f;
        for (int k = min_bin; k <= max_bin; ++k) {
            if (k < channel_ffts[0].size()) {
                target_signal_energy += std::norm(channel_ffts[0][k]); 
            }
            if (k < g_noise_floor_spectrum.size()) {
                target_noise_energy += g_noise_floor_spectrum[k];
            }
        }

        if (target_signal_energy > target_noise_energy * NOISE_THRESHOLD_MULTIPLIER) {
            // --- Sound Detected ---
            auto result = calculate_doa_powers(channel_ffts, all_steering_vectors, g_noise_floor_spectrum);
            int peak_angle_index = result.first;
            float final_angle_deg = -1.0f;

            if (peak_angle_index != -1) {
                auto angle_powers = result.second;
                int prev_index = (peak_angle_index == 0) ? NUM_ANGLES - 1 : peak_angle_index - 1;
                int next_index = (peak_angle_index == NUM_ANGLES - 1) ? 0 : peak_angle_index + 1;
                final_angle_deg = parabolic_interpolation(angle_powers[prev_index], angle_powers[peak_angle_index], angle_powers[next_index], peak_angle_index);
            }

            float current_rms_energy = calculate_rms_energy(current_frame);

            // --- DISTANCE CALCULATION ---
            float estimated_dist = estimate_distance(current_rms_energy);

            data_packet["doa"] = final_angle_deg;
            data_packet["amplitude"] = std::min(1.0f, current_rms_energy / (REFERENCE_ENERGY * 2.0f));
            data_packet["distance"] = estimated_dist;
            data_packet["waveform"] = normalize_waveform(current_frame[0]);
            data_packet["freq_bins"] = get_freq_bins(channel_ffts[0]);

        } else {
            // --- No significant sound detected ---
            data_packet["doa"] = 0.0;
            data_packet["amplitude"] = 0.0;
            data_packet["distance"] = -1.0; // Indicate no detection/Too Quiet
            data_packet["waveform"] = std::vector<float>();
            data_packet["freq_bins"] = std::vector<float>();
        }

        broadcast_json(data_packet);

        auto start_time = std::chrono::steady_clock::now();
        while (g_running && std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(LOOP_SLEEP_MS)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "Server shutting down." << std::endl;
    ma_device_stop(&device);
    ma_device_uninit(&device);
    
    g_ws_server->stop();
    
    if (ws_thread.joinable()) {
        ws_thread.join();
    }
    
    std::cout << "Shutdown complete." << std::endl;
    return 0;
}


// =================================================================================================
//  Helper Function Implementations
// =================================================================================================

void read_new_hop(UserData& userData, std::vector<float>& dest_buffer) {
    while (g_running) {
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
    if (frame.empty() || frame[0].empty()) return 0.0f;
    for (float sample : frame[0]) {
        sum_sq += sample * sample;
    }
    return std::sqrt(sum_sq / frame[0].size());
}

// --- NEW: Distance Estimation Helper ---
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

std::pair<int, std::vector<float>> calculate_doa_powers(
    std::vector<ComplexVector>& channel_ffts,
    const std::vector<SteeringVector>& all_steering_vectors,
    const std::vector<float>& noise_floor)
{
    std::vector<float> angle_powers(NUM_ANGLES, 0.0f);
    int best_angle_index = -1;
    float max_power = -1.0f;

    const int min_bin = static_cast<int>(MIN_FREQ * FFT_SIZE / SAMPLE_RATE);
    const int max_bin = static_cast<int>(MAX_FREQ * FFT_SIZE / SAMPLE_RATE);
    const float gaussian_std_dev = 0.5f * FREQ_BANDWIDTH;

    for (auto& fft_vec : channel_ffts) {
        for (size_t k = 0; k < fft_vec.size() && k < noise_floor.size(); ++k) {
            float power = std::norm(fft_vec[k]);
            float noise = noise_floor[k];
            float new_mag = 0.0f;
            
            if (power > noise * NOISE_THRESHOLD_MULTIPLIER) {
                new_mag = std::sqrt(power - (noise * (NOISE_THRESHOLD_MULTIPLIER -1) ));
            }
            
            fft_vec[k] *= (new_mag / (std::sqrt(power) + 1e-9f));
            
            float bin_freq = (float)k * SAMPLE_RATE / FFT_SIZE;
            float distance = std::abs(bin_freq - TARGET_FREQ);
            float weight = std::exp(-0.5f * std::pow(distance / gaussian_std_dev, 2.0f));
            
            fft_vec[k] *= weight;
        }
    }
    
    for (int i = 0; i < NUM_ANGLES; ++i) {
        ComplexVector summed_spectrum(FFT_SIZE / 2 + 1, {0.0f, 0.0f});
        for (int mic = 1; mic <= 6; ++mic) {
            for (int k = min_bin; k <= max_bin; ++k) {
                if(k < summed_spectrum.size() && k < channel_ffts[mic].size() && k < all_steering_vectors[i][mic].size()) {
                    summed_spectrum[k] += channel_ffts[mic][k] * std::conj(all_steering_vectors[i][mic][k]);
                }
            }
        }
        
        float current_power = 0.0f;
        for (int k = min_bin; k <= max_bin; ++k) {
            if(k < summed_spectrum.size()) {
                current_power += std::norm(summed_spectrum[k]);
            }
        }
        
        angle_powers[i] = current_power;
        if (current_power > max_power) {
            max_power = current_power;
            best_angle_index = i;
        }
    }
    return {best_angle_index, angle_powers};
}


// --- UI HELPER FUNCTIONS ---
std::vector<float> normalize_waveform(const std::vector<float>& wave) {
    std::vector<float> normalized_wave = wave;
    float max_abs = 1e-6f;
    for (float sample : wave) {
        if (std::abs(sample) > max_abs) {
            max_abs = std::abs(sample);
        }
    }
    if (max_abs > 1e-5f) {
        for (float& sample : normalized_wave) {
            sample /= max_abs;
        }
    }
    if (normalized_wave.size() > 256) {
        normalized_wave.resize(256);
    }
    return normalized_wave;
}

std::vector<float> get_freq_bins(const ComplexVector& fft_data) {
    const int num_bins_to_send = 64;
    std::vector<float> bins(num_bins_to_send);
    
    const float display_range = FREQ_BANDWIDTH * 2.0f;
    const float display_min_freq = std::max(0.0f, TARGET_FREQ - display_range);
    const float display_max_freq = std::min((float)SAMPLE_RATE / 2.0f, TARGET_FREQ + display_range);

    const int min_bin_idx = static_cast<int>(display_min_freq * FFT_SIZE / SAMPLE_RATE);
    const int max_bin_idx = static_cast<int>(display_max_freq * FFT_SIZE / SAMPLE_RATE);
    
    int num_bins_in_range = max_bin_idx - min_bin_idx;
    if (num_bins_in_range <= 0) return bins;
    
    float max_magnitude = 1e-6f;

    for (int i = 0; i < num_bins_to_send; ++i) {
        int fft_index = min_bin_idx + static_cast<int>( (float)i / num_bins_to_send * num_bins_in_range );
        if(fft_index >= 0 && fft_index < fft_data.size()) {
            float magnitude = std::abs(fft_data[fft_index]);
            bins[i] = magnitude;
            if (magnitude > max_magnitude) {
                max_magnitude = magnitude;
            }
        }
    }

    if (max_magnitude > 1e-5f) {
        for (float& bin : bins) {
            bin /= max_magnitude;
        }
    }
    return bins;
}