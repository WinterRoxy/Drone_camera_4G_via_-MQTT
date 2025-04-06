# Drone Communication & Image Processing System

This repository contains the firmware and software components for a drone control and image processing system using ESP32 microcontrollers and MQTT. The system consists of several modules that work together to:
  
- **Control and monitor the drone’s flight parameters** (e.g., angles, battery status).
- **Transmit control commands** from a remote transmitter to the drone.
- **Capture images** from an onboard camera.
- **Relay captured images** via a cellular modem (SIM7600) to an MQTT broker.
- **Receive and process images** on a PC for human detection using a pre-trained YOLO model.

Below is a summary of the key files and their functions:

---


## Operating Mechanism
![Image](https://github.com/user-attachments/assets/8bc3b1ff-338c-4e18-97aa-41f086786e05)
1. **Flight Control & Telemetry:**
   - The **`esp_flight_controller`** on the drone receives remote control commands (sent by `esp_remote_tx_rx`) and measures the drone’s flight parameters.
   - It sends back telemetry (e.g., orientation and battery status) to the remote controller, which then forwards the information to ESP RainMaker for remote monitoring.
   - The system processes control signals received from the controller to navigate and stabilize the drone. The ESP32 recieve tilt angles and angular velocity data from the MPU6050 sensor to determine whether the drone is tilting or unbalanced. Based on this information, it adjusts the rotation speed of the corresponding propellers to restore balance
   - When directional movement is desired, the ESP32 maps the received control signals to a specific tilt angle and controls the propeller speeds accordingly to tilt the drone in that direction, instead of maintaining level balance as before

2. **Image Capture & Transmission:**
   - The **`Broadcast_master_esp_cam`** module on the ESP32-CAM captures images and splits the data into packets.
   - It broadcasts these packets via the ESP-NOW protocol to the **`Broadcast_slave`** module on another ESP32 aboard the drone.

3. **Image Upload via Cellular:**
   - The **`Broadcast_slave`** module reassembles the received image data and establishes a PPP connection using the SIM7600C module.
   - It then uploads the complete image to an MQTT broker over a 4G network.

4. **Image Processing on PC:**
   - The **`image_reciever.py`** script subscribes to the MQTT topic and receives the uploaded image.
   - The image is saved locally and processed by a YOLO model (loaded from **`best_train_800x600.pt`**) to detect persons.
   - The output (e.g., the number of detected people) can be used for further decision-making or alerts.

---

## Dependencies

- **For ESP32 firmware (esp_flight_controller, esp_remote_tx_rx, Broadcast_master_esp_cam, Broadcast_slave):**
  - ESP-IDF or Arduino framework for ESP32
  - Libraries: ESP-NOW, PPP library, WiFi, and camera library (for ESP32-CAM)
  - SIM7600C modem library for PPPOS

- **For Python image processing (image_reciever.py):**
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
   - Ensure the MQTT broker settings in `image_reciever.py` are correctly configured.
   - Run `image_reciever.py` on your PC. The script will subscribe to the topic and process incoming images using the YOLO model (`best_train_800x600.pt`).

3. **Monitoring:**
   - Use Serial Monitor on the ESP32 devices for debugging and status messages.
   - Monitor MQTT topics to view published telemetry and image data.

---

## Summary

This system integrates multiple ESP32 devices and a cellular modem to enable robust drone operation and remote monitoring. The drone’s flight controller handles real-time telemetry and control signals, while a dedicated remote unit manages command transmission. Image capture is performed by an ESP32-CAM which sends data via ESP-NOW to a secondary ESP32 that relays images over a 4G network to an MQTT broker. A PC-based Python script then receives, saves, and processes these images using a YOLO model for human detection. This modular design facilitates flexible deployment and remote management of drone operations.

---
