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
