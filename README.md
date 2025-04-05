# Drone Communication & Image Processing System

This repository contains the firmware and software components for a drone control and image processing system using ESP32 microcontrollers and MQTT. The system consists of several modules that work together to:
  
- **Control and monitor the drone’s flight parameters** (e.g., angles, battery status).
- **Transmit control commands** from a remote transmitter to the drone.
- **Capture images** from an onboard camera.
- **Relay captured images** via a cellular modem (SIM7600) to an MQTT broker.
- **Receive and process images** on a PC for human detection using a pre-trained YOLO model.

Below is a summary of the key files and their functions:

---

## File Descriptions

### 1. `esp_flight_controller`
- **Purpose:**  
  This firmware is loaded onto an ESP32 placed on the drone. It receives control signals (e.g., from the remote transmitter) and returns telemetry data such as the drone's orientation and battery status.
- **Functionality:**  
  - Interprets incoming control commands.
  - Monitors flight parameters (angles, battery level).
  - Sends telemetry data back to the remote unit.

---

### 2. `esp_remote_tx_rx`
- **Purpose:**  
  This firmware is designed for an ESP32-S3 used as a remote transmitter/receiver. It sends control signals to the drone and receives telemetry data from it, then forwards these parameters to ESP RainMaker for further processing and visualization.
- **Functionality:**  
  - Transmits control commands to the drone.
  - Receives flight telemetry (angles, battery status) from the drone.
  - Interfaces with ESP RainMaker to update remote status.

---

### 3. `Broadcast_master_esp_cam`
- **Purpose:**  
  This firmware is for an ESP32-CAM module configured to capture images. It sets up an ESP-NOW protocol to broadcast captured images to a second ESP32 on the drone.
- **Functionality:**  
  - Configures the camera and captures images.
  - Uses ESP-NOW to send image data packets to the slave device on the drone.
  - Handles the segmentation and transmission of large image files.

---

### 4. `Broadcast_slave`
- **Purpose:**  
  This firmware runs on the second ESP32 (Slave) placed on the drone. It receives image data from the ESP32-CAM via ESP-NOW, then uses a SIM7600C module (with PPPOS) to upload the image to an MQTT broker over the 4G cellular network.
- **Functionality:**  
  - Receives segmented image data via ESP-NOW.
  - Reassembles the image data.
  - Establishes a PPP connection via the SIM7600C modem.
  - Publishes the complete image to an MQTT broker using mobile network connectivity.

---

### 5. `demo.py`
- **Purpose:**  
  This Python script runs on a PC and subscribes to the MQTT topic where the drone’s Slave device publishes the captured images. It decodes and saves the received image for further processing.
- **Functionality:**  
  - Connects to an MQTT broker (e.g., test.mosquitto.org).
  - Receives image segments and reconstructs the full image.
  - Saves the image locally.
  - (Optionally) Performs further image processing (e.g., human detection).

---

### 6. `best_train_800x600.pt`
- **Purpose:**  
  This is a pre-trained YOLO model file used for image analysis (e.g., detecting people in the captured images). The model processes images received via MQTT (from `demo.py`).
- **Functionality:**  
  - Loaded by the image processing application (demo.py).
  - Performs object detection to identify persons in the images.
  - Can be integrated into further automated processing or alert systems.

---

## Operating Mechanism

1. **Flight Control & Telemetry:**
   - The **`esp_flight_controller`** on the drone receives remote control commands (sent by `esp_remote_tx_rx`) and measures the drone’s flight parameters.
   - It sends back telemetry (e.g., orientation and battery status) to the remote controller, which then forwards the information to ESP RainMaker for remote monitoring.

2. **Image Capture & Transmission:**
   - The **`Broadcast_master_esp_cam`** module on the ESP32-CAM captures images and splits the data into packets.
   - It broadcasts these packets via the ESP-NOW protocol to the **`Broadcast_slave`** module on another ESP32 aboard the drone.

3. **Image Upload via Cellular:**
   - The **`Broadcast_slave`** module reassembles the received image data and establishes a PPP connection using the SIM7600C module.
   - It then uploads the complete image to an MQTT broker over a 4G network.

4. **Image Processing on PC:**
   - The **`demo.py`** script subscribes to the MQTT topic and receives the uploaded image.
   - The image is saved locally and processed by a YOLO model (loaded from **`best_train_800x600.pt`**) to detect persons.
   - The output (e.g., the number of detected people) can be used for further decision-making or alerts.

---

## Dependencies

- **For ESP32 firmware (esp_flight_controller, esp_remote_tx_rx, Broadcast_master_esp_cam, Broadcast_slave):**
  - ESP-IDF or Arduino framework for ESP32
  - Libraries: ESP-NOW, PPP library, WiFi, and camera library (for ESP32-CAM)
  - SIM7600C modem library for PPPOS

- **For Python image processing (demo.py):**
  - Python 3.x
  - `paho-mqtt` for MQTT communication
  - `roboflow` (if using Roboflow integration)
  - `ultralytics` (YOLO model inference)
  - Standard libraries: time, os, threading

---

## Usage

1. **Drone Firmware:**
   - Flash **`esp_flight_controller`** onto the drone’s primary ESP32.
   - Flash **`esp_remote_tx_rx`** onto the remote ESP32-S3 controller.
   - Flash **`Broadcast_master_esp_cam`** onto the ESP32-CAM module.
   - Flash **`Broadcast_slave`** onto the second ESP32 on the drone that is connected to the SIM7600C modem.

2. **Image Processing:**
   - Ensure the MQTT broker settings in `demo.py` are correctly configured.
   - Run `demo.py` on your PC. The script will subscribe to the topic and process incoming images using the YOLO model (`best_train_800x600.pt`).

3. **Monitoring:**
   - Use Serial Monitor on the ESP32 devices for debugging and status messages.
   - Monitor MQTT topics to view published telemetry and image data.

---

## Summary

This system integrates multiple ESP32 devices and a cellular modem to enable robust drone operation and remote monitoring. The drone’s flight controller handles real-time telemetry and control signals, while a dedicated remote unit manages command transmission. Image capture is performed by an ESP32-CAM which sends data via ESP-NOW to a secondary ESP32 that relays images over a 4G network to an MQTT broker. A PC-based Python script then receives, saves, and processes these images using a YOLO model for human detection. This modular design facilitates flexible deployment and remote management of drone operations.

---
