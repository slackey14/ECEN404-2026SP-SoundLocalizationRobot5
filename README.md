# Sound Localization Robot 5, Team 23

**Sound Rover** is an autonomous, omnidirectional robot powered by an ESP32-S3 Wroom. It uses a 4-microphone array to localize target sound frequencies via Phase-Gradient TDOA (SRP-PHAT beamforming) and navigates toward them using A* path planning, mecanum wheels, and ultrasonic obstacle avoidance.

## Features
* **Acoustic Localization:** 4-mic SRP-PHAT beamforming for 360° sound direction finding and distance estimation.
* **Omnidirectional Drive:** Mecanum wheel control for complex maneuvering and strafing.
* **Autonomous Navigation:** Real-time A* path planning, mapping, and destination re-calculation on the fly.
* **Obstacle Avoidance:** 4x Ultrasonic sensors (Front, Back, Left, Right) via hardware-driven edge interrupts.
* **Web Interface:** Built-in WiFi Access Point and HTTP server providing a live web UI with map visualization, manual controls, and live DSP configuration.

## Hardware Architecture
* **Microcontroller:** ESP32-S3 WROOM 1
* **Microphones:** 4x I2S Digital Microphones
* **Sensors:** 6x Ultrasonic Distance Sensors
* **Chassis:** 4WD Mecanum Wheel chassis with independent motor drivers (PWM/Dir)

## Software Stack
* **Framework:** ESP-IDF / FreeRTOS (via PlatformIO)
* **Concurrency:** * **Core 0:** WiFi AP, HTTP Server, and Sound DSP Task (I2S DMA + FFT + SRP-PHAT).
  * **Core 1:** Navigation Task (A* Routing, Motor Control, Obstacle Detection).

## Getting Started
1. Flash the firmware to your ESP32-S3 using PlatformIO.
2. Power on the Rover.
3. Connect your device to the newly created WiFi Access Point: `Sound_Rover` (Open network).
4. Open a web browser and navigate to `http://192.168.4.1`.
5. From the Web UI, you can manually drive the rover, view the live localized obstacle map, or click **Chase Sound Source** to begin autonomous acoustic tracking.
