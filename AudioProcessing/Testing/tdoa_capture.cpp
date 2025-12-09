// =================================================================================================
// UMA-8 Multi-Channel Audio Capture & Data Exporter for TDOA in C++
// =================================================================================================
//
// Description:
// This program captures 8 channels of audio and saves the raw sample data to a CSV file.
// This CSV file can then be easily plotted and analyzed by other programs, like a Python script.
//
// Dependencies:
// - miniaudio: A single-file audio library. Just download "miniaudio.h" and place it
//   in the same directory as this source file.
//   Get it here: https://miniaud.io/
//
// Compilation (Linux/macOS):
// g++ -std=c++17 tdoa_capture.cpp -o tdoa_capture -lpthread
//
// Usage:
// ./tdoa_capture
//
// Author: Gemini
// =================================================================================================

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <string>
#include <algorithm> // For std::max_element
#include <fstream>   // For file writing

// --- Configuration ---
const int SAMPLE_RATE = 48000;
const int CHANNEL_COUNT = 8;
const int CAPTURE_DURATION_MS = 10000; // Capture for 10 seconds for a more manageable file size
const std::string OUTPUT_FILENAME = "uma8_capture.csv";

// --- Global Data Structures ---
struct UserData {
    std::vector<std::vector<float>> audio_channels;
    std::mutex data_mutex;

    UserData() : audio_channels(CHANNEL_COUNT) {}
};

// =================================================================================================
//  Save Audio Data to CSV File
// =================================================================================================
void save_audio_to_csv(const std::vector<std::vector<float>>& audio_data) {
    std::cout << "\n--- Saving captured audio to " << OUTPUT_FILENAME << " ---" << std::endl;
    if (audio_data.empty() || audio_data[0].empty()) {
        std::cout << "No audio data to save." << std::endl;
        return;
    }

    std::ofstream output_file(OUTPUT_FILENAME);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open file " << OUTPUT_FILENAME << " for writing." << std::endl;
        return;
    }

    // Write the header row
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        output_file << "Channel_" << i << (i == CHANNEL_COUNT - 1 ? "" : ",");
    }
    output_file << "\n";

    // Write the sample data row by row
    size_t num_samples = audio_data[0].size();
    for (size_t i = 0; i < num_samples; ++i) {
        for (int j = 0; j < CHANNEL_COUNT; ++j) {
            output_file << audio_data[j][i] << (j == CHANNEL_COUNT - 1 ? "" : ",");
        }
        output_file << "\n";
    }

    output_file.close();
    std::cout << "Successfully saved " << num_samples << " samples for each of the " << CHANNEL_COUNT << " channels." << std::endl;
}


// =================================================================================================
//  Audio Callback Function
// =================================================================================================
void data_callback(ma_device* pDevice, void* pOutput, const void* pInput, ma_uint32 frameCount) {
    (void)pOutput; // Unused for capture
    UserData* pUserData = (UserData*)pDevice->pUserData;
    if (pUserData == nullptr) return;

    const float* pInputF32 = (const float*)pInput;
    std::lock_guard<std::mutex> lock(pUserData->data_mutex);

    for (ma_uint32 i = 0; i < frameCount; ++i) {
        for (int j = 0; j < CHANNEL_COUNT; ++j) {
            pUserData->audio_channels[j].push_back(pInputF32[i * CHANNEL_COUNT + j]);
        }
    }
}

// =================================================================================================
//  Main Function
// =================================================================================================
int main() {
    ma_result result;
    ma_context context;
    ma_device_info* pCaptureDeviceInfos;
    ma_uint32 captureDeviceCount;
    ma_device_config deviceConfig;
    ma_device device;
    UserData userData;

    if (ma_context_init(NULL, 0, NULL, &context) != MA_SUCCESS) {
        std::cerr << "Failed to initialize context." << std::endl;
        return -1;
    }

    result = ma_context_get_devices(&context, NULL, NULL, &pCaptureDeviceInfos, &captureDeviceCount);
    if (result != MA_SUCCESS) {
        std::cerr << "Failed to retrieve device information." << std::endl;
        ma_context_uninit(&context);
        return -1;
    }

    std::cout << "Available Capture Devices:" << std::endl;
    ma_uint32 umaDeviceIndex = -1;
    for (ma_uint32 i = 0; i < captureDeviceCount; i++) {
        std::cout << "  " << i << ": " << pCaptureDeviceInfos[i].name << std::endl;
        if (strstr(pCaptureDeviceInfos[i].name, "UMA-8") != NULL) {
             umaDeviceIndex = i;
        }
    }

    ma_uint32 selectedDeviceIndex;
    if (umaDeviceIndex != (ma_uint32)-1) {
        std::cout << "UMA-8 found, auto-selecting index " << umaDeviceIndex << "." << std::endl;
        selectedDeviceIndex = umaDeviceIndex;
    } else {
        std::cout << "Please select a device index: ";
        std::cin >> selectedDeviceIndex;
        if (selectedDeviceIndex >= captureDeviceCount) {
            std::cerr << "Invalid device index." << std::endl;
            ma_context_uninit(&context);
            return -1;
        }
    }

    std::cout << "Initializing Microphone Capture..." << std::endl;

    deviceConfig = ma_device_config_init(ma_device_type_capture);
    deviceConfig.capture.pDeviceID = &pCaptureDeviceInfos[selectedDeviceIndex].id;
    deviceConfig.capture.format    = ma_format_f32;
    deviceConfig.capture.channels  = CHANNEL_COUNT;
    deviceConfig.sampleRate        = SAMPLE_RATE;
    deviceConfig.dataCallback      = data_callback;
    deviceConfig.pUserData         = &userData;

    result = ma_device_init(&context, &deviceConfig, &device);
    if (result != MA_SUCCESS) {
        std::cerr << "Failed to initialize capture device. Error: " << ma_result_description(result) << std::endl;
        ma_context_uninit(&context);
        return -1;
    }
    
    std::cout << "Device Name: " << pCaptureDeviceInfos[selectedDeviceIndex].name << std::endl;

    result = ma_device_start(&device);
    if (result != MA_SUCCESS) {
        ma_device_uninit(&device);
        ma_context_uninit(&context);
        std::cerr << "Failed to start device. Error: " << ma_result_description(result) << std::endl;
        return -1;
    }

    std::cout << "Recording for " << CAPTURE_DURATION_MS << " ms... (Try making some noise!)" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(CAPTURE_DURATION_MS));

    ma_device_uninit(&device);
    ma_context_uninit(&context);

    std::cout << "Recording finished." << std::endl;

    {
        std::lock_guard<std::mutex> lock(userData.data_mutex);
        save_audio_to_csv(userData.audio_channels);
    }

    std::cout << "\nTo visualize the data, run the Python script: python plot_waveforms.py" << std::endl;

    return 0;
}

