#define _USE_MATH_DEFINES //added due to math error
#include <cmath>
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "fft.hpp" //
#include <fstream> //For writing possible python file

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

// --- Configuration ---
const int SAMPLE_RATE = 48000;
const int CHANNEL_COUNT = 8;
const float SPEED_OF_SOUND = 343.0f; // meters per second
const float MIC_RADIUS = 0.045f;     // 45mm for UMA-8

// --- TDOA Processing Configuration ---
const int FFT_SIZE = 1024;
const int HOP_SIZE = FFT_SIZE / 2;
const float ENERGY_THRESHOLD = 0.001f; // Adjust based on sensitivity needs
const double VOICE_FREQ_GAIN = 10.0; // Boosts voice frequencies by 3x


// --- Bandpass Filter Configuration for Human Voice ---
const float MIN_FREQ = 300.0f;  // Minimum frequency for human voice
const float MAX_FREQ = 3400.0f; // Maximum frequency for human voice

// --- Type definitions for clarity ---
using Complex = std::complex<double>;
using ComplexVector = std::vector<Complex>;
using SteeringVector = std::vector<ComplexVector>; // [mic_index][freq_bin]

// --- Global Data Structures ---
struct UserData {
    std::vector<float> audio_buffer;
    std::mutex buffer_mutex;
    size_t head = 0;
};

const std::vector<std::pair<float, float>> MIC_POSITIONS = {
    {0.0f, 0.0f}, //Mic 0 (center) - Not used in DOA
    {MIC_RADIUS * cosf(0.0f * M_PI / 180.0f), MIC_RADIUS * sinf(0.0f * M_PI / 180.0f)},   // Mic 1 (0 deg)
    {MIC_RADIUS * cosf(60.0f * M_PI / 180.0f), MIC_RADIUS * sinf(60.0f * M_PI / 180.0f)},  // Mic 2 (60 deg)
    {MIC_RADIUS * cosf(120.0f * M_PI / 180.0f), MIC_RADIUS * sinf(120.0f * M_PI / 180.0f)},// Mic 3 (120 deg)
    {MIC_RADIUS * cosf(180.0f * M_PI / 180.0f), MIC_RADIUS * sinf(180.0f * M_PI / 180.0f)},// Mic 4 (180 deg)
    {MIC_RADIUS * cosf(240.0f * M_PI / 180.0f), MIC_RADIUS * sinf(240.0f * M_PI / 180.0f)},// Mic 5 (240 deg)
    {MIC_RADIUS * cosf(300.0f * M_PI / 180.0f), MIC_RADIUS * sinf(300.0f * M_PI / 180.0f)},// Mic 6 (300 deg)
    {0.0f, 0.0f}, //Mic 7 (spare)
};

// Pre-computes the phase shifts for all angles, mics, and frequencies
std::vector<SteeringVector> precompute_steering_vectors() {
    std::vector<SteeringVector> all_steering_vectors(360);

    for (int angle = 0; angle < 360; ++angle) {
        all_steering_vectors[angle].resize(CHANNEL_COUNT);
        // Use double for all calculations
        double angle_rad = angle * M_PI / 180.0;

        for (int i = 1; i <= 6; ++i) { // Only for the 6 outer mics
            all_steering_vectors[angle][i].resize(FFT_SIZE / 2 + 1);
            // Ensure MIC_POSITIONS values are treated as double
            double mic_x = MIC_POSITIONS[i].first;
            double mic_y = MIC_POSITIONS[i].second;

            // Project mic position onto the sound wave direction vector
            double projection = mic_x * cos(angle_rad) + mic_y * sin(angle_rad);
            double time_delay = projection / SPEED_OF_SOUND;

            for (int k = 0; k <= FFT_SIZE / 2; ++k) {
                double freq = (double)k * SAMPLE_RATE / FFT_SIZE;
                double omega = 2.0 * M_PI * freq;
                // The steering vector is the complex exponential representing the phase shift
                // This line will now work correctly
                all_steering_vectors[angle][i][k] = std::exp(Complex(0.0, 1.0) * omega * time_delay);
            }
        }
    }
    return all_steering_vectors;
}

// UPDATED ALGORITHM: Frequency-Domain Beamforming with Voice Amplification
std::pair<int, double> calculate_doa_fft(
    std::vector<ComplexVector>& channel_ffts,
    const std::vector<SteeringVector>& all_steering_vectors) {

    double max_power = -1.0;
    int best_angle = -1;

    // Define the frequency bins for our bandpass filter
    const int min_bin = static_cast<int>(MIN_FREQ * FFT_SIZE / SAMPLE_RATE);
    const int max_bin = static_cast<int>(MAX_FREQ * FFT_SIZE / SAMPLE_RATE);

    // Apply bandpass filter AND amplify voice frequencies
    for (auto& fft_vec : channel_ffts) {
        for (int k = 0; k < fft_vec.size(); ++k) {
            if (k >= min_bin && k <= max_bin) {
                // Apply gain to the frequencies we want
                fft_vec[k] *= VOICE_FREQ_GAIN;
            } else {
                // Zero out all other frequencies
                fft_vec[k] = 0;
            }
        }
    }

    // --- The rest of the function remains the same ---
    for (int angle = 0; angle < 360; ++angle) {
        ComplexVector summed_spectrum(FFT_SIZE / 2 + 1, {0.0, 0.0});

        for (int i = 1; i <= 6; ++i) { // Only use the 6 outer mics
            for (int k = min_bin; k <= max_bin; ++k) {
                summed_spectrum[k] += channel_ffts[i][k] * std::conj(all_steering_vectors[angle][i][k]);
            }
        }

        double current_power = 0.0;
        for (int k = min_bin; k <= max_bin; ++k) {
            current_power += std::norm(summed_spectrum[k]);
        }

        if (current_power > max_power) {
            max_power = current_power;
            best_angle = angle;
        }
    }
    return {best_angle, max_power};
}

// Function to print the debug dashboard (no changes needed)
void print_debug_dashboard(float rms_energy, int final_angle, float beam_energy) {
     // Clear the screen in a portable way
    #ifdef _WIN32
        system("cls");
    #else
        // Move cursor to top-left corner and clear screen using ANSI codes for Linux/macOS
        std::cout << "\033[2J\033[H";
    #endif
    
    std::cout << "===== UMA-8 TDOA Real-Time Debug Dashboard (Optimized) =====\n";
    std::cout << "Listening for human voice (" << MIN_FREQ << "-" << MAX_FREQ << " Hz)...\n";
    std::cout << "------------------------------------------------\n";
    
    std::cout << "RMS Energy: " << std::fixed << std::setprecision(4) << rms_energy 
              << " (Threshold: " << ENERGY_THRESHOLD << ")" << (rms_energy >= ENERGY_THRESHOLD ? " [SOUND DETECTED]" : " [SILENT]")
              << "       \n";
    
    std::cout << "------------------------------------------------\n";
    std::cout << "Final Estimated Angle: " << (final_angle >= 0 ? std::to_string(final_angle) : "N/A") << " degrees            \n";
    std::cout << "Beamformer Power:      " << (final_angle >= 0 ? std::to_string(beam_energy) : "N/A") << " (Higher is better)\n";

    // ASCII Visualizer
    std::string compass_line(45, ' ');
    if (final_angle >= 0) {
        int pos = static_cast<int>(round((final_angle / 360.0) * 44.0));
        compass_line[pos] = 'V';
    }
    std::cout << "\n 0" << std::string(20, '-') << "180" << std::string(20, '-') << "359\n";
    std::cout << "[" << compass_line << "]\n";
    
    std::cout << "\nPress Enter to quit.\n" << std::flush;
}

// Saves the captured multi-channel audio frame to a CSV file
void save_capture_to_csv(const std::vector<std::vector<double>>& channels) {
    static int capture_count = 0; // Static counter to create unique filenames
    std::string filename = "capture_" + std::to_string(capture_count++) + ".csv";
    
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write header
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        csv_file << "Mic" << i << (i == CHANNEL_COUNT - 1 ? "" : ",");
    }
    csv_file << "\n";

    // Write data rows (sample by sample)
    for (int i = 0; i < FFT_SIZE; ++i) {
        for (int j = 0; j < CHANNEL_COUNT; ++j) {
            csv_file << channels[j][i] << (j == CHANNEL_COUNT - 1 ? "" : ",");
        }
        csv_file << "\n";
    }

    std::cout << "Saved capture to " << filename << std::endl;
}

// data_callback remains the same
void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    (void)pOutput;
    UserData* pUserData = (UserData*)pDevice->pUserData;
    if (pUserData == nullptr) return;
    const float* pInputF32 = (const float*)pInput;
    
    std::lock_guard<std::mutex> lock(pUserData->buffer_mutex);
    for (ma_uint32 i = 0; i < frameCount * CHANNEL_COUNT; ++i) {
        pUserData->audio_buffer[pUserData->head] = pInputF32[i];
        pUserData->head = (pUserData->head + 1) % pUserData->audio_buffer.size();
    }
}


// =================================================================================================
//  Main Function
// =================================================================================================
int main() {
    // --- Pre-computation Step ---
    std::cout << "Pre-computing steering vectors..." << std::endl;
    auto all_steering_vectors = precompute_steering_vectors();
    std::cout << "Done." << std::endl;

    UserData userData;
    // Buffer for 2 seconds of audio
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
    std::vector<double> process_buffer(FFT_SIZE * CHANNEL_COUNT);
    // Create a Hamming window for better FFT results
    std::vector<double> window(FFT_SIZE);
    for(int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.54f - 0.46f * cosf(2.0f * M_PI * i / (FFT_SIZE - 1));
    }


    while (true) {
        if (std::cin.rdbuf()->in_avail() > 0) break;

        size_t captured_head;
        {
            std::lock_guard<std::mutex> lock(userData.buffer_mutex);
            captured_head = userData.head;
        }
        
        // Wait until we have a new hop of audio data
        if ((captured_head - processing_head + userData.audio_buffer.size()) % userData.audio_buffer.size() >= HOP_SIZE * CHANNEL_COUNT) {
            
            // --- Copy a full frame (FFT_SIZE) of data from the circular buffer ---
            {
                std::lock_guard<std::mutex> lock(userData.buffer_mutex);
                size_t start_pos = (processing_head - (FFT_SIZE / 2 * CHANNEL_COUNT) + userData.audio_buffer.size()) % userData.audio_buffer.size();
                for (int i = 0; i < FFT_SIZE * CHANNEL_COUNT; ++i) {
                   process_buffer[i] = userData.audio_buffer[(start_pos + i) % userData.audio_buffer.size()];
                }
            }
            processing_head = (processing_head + HOP_SIZE * CHANNEL_COUNT) % userData.audio_buffer.size();
            
            // --- De-interleave and window the audio data ---
            std::vector<std::vector<float>> channels(CHANNEL_COUNT, std::vector<float>(FFT_SIZE));
            for(int i = 0; i < FFT_SIZE; ++i) {
                for(int j = 0; j < CHANNEL_COUNT; ++j) {
                    channels[j][i] = process_buffer[i * CHANNEL_COUNT + j] * window[i];
                }
            }

            // --- Check energy threshold ---
            float rms_energy = 0.0f;
            for (float sample : channels[0]) rms_energy += sample * sample; // Use central mic for energy check
            rms_energy = std::sqrt(rms_energy / channels[0].size());
            
            int final_angle = -1;
            float beam_energy = 0.0f;

            if (rms_energy >= ENERGY_THRESHOLD) {
                // --- Perform FFT on all channels ---
                std::vector<ComplexVector> channel_ffts(CHANNEL_COUNT);
                for (int i = 0; i < CHANNEL_COUNT; ++i) {
                    // 1. Copy the real-valued channel data into a complex vector
                    channel_ffts[i].assign(channels[i].begin(), channels[i].end());

                    // 2. Perform the in-place FFT on the complex vector
                    Fft::transform(channel_ffts[i]);
                }

                // --- Run the localization algorithm ---
                auto result = calculate_doa_fft(channel_ffts, all_steering_vectors);
                final_angle = result.first;
                beam_energy = result.second;
            }
            
            print_debug_dashboard(rms_energy, final_angle, beam_energy);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nStopping device..." << std::endl;
    ma_device_uninit(&device);
    return 0;
}