import paho.mqtt.client as mqtt
import time
import os
import threading
from roboflow import Roboflow
from ultralytics import YOLO

# MQTT Settings
mqtt_broker = "test.mosquitto.org"
mqtt_port = 1883
mqtt_topic = "drone/send/pic"

# Load YOLO model (trained with Roboflow or custom dataset)
model = YOLO("best_train_800x600.pt")

# Variables to store incoming image data
image_data = bytearray()
image_ready = False
lock = threading.Lock()

# Callback when a message is received from MQTT
def on_message(client, userdata, msg):
    global image_data, image_ready
    segment = msg.payload  # Binary data

    with lock:
        image_data += segment

    # Check if it's the last segment (assuming last packet < 1024 bytes)
    if len(segment) < 1024:
        image_ready = True
        print("Received complete image.")

# Function to handle and process the full image
def process_image():
    global image_ready, image_data
    while True:
        if image_ready:
            with lock:
                full_image = bytes(image_data)
                image_data = bytearray()
                image_ready = False
            
            # Save image to a temporary file
            temp_filename = "temp_image.jpg"
            try:
                with open(temp_filename, "wb") as img_file:
                    img_file.write(full_image)
                print(f"Image saved temporarily as {temp_filename}")
                
                # Run YOLO detection and save result
                results = model.predict(temp_filename, save=True, project="received_images", exist_ok=True)
                
                # Count number of detected people (class ID = 0)
                per_cnt = 0
                for result in results:
                    for box in result.boxes.data:
                        cls = int(box[-1])
                        if cls == 0:
                            per_cnt += 1
                print(f"Number of detected people: {per_cnt}")
            except Exception as e:
                print(f"Error processing image: {e}")
            finally:
                # Delete temp file
                if os.path.exists(temp_filename):
                    os.remove(temp_filename)

# Setup MQTT
client = mqtt.Client()
client.on_message = on_message

try:
    client.connect(mqtt_broker, mqtt_port, 60)
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
    exit(1)

client.subscribe(mqtt_topic)

# Start MQTT loop in a background thread so it doesn’t block the main thread
client.loop_start()
print("Waiting for image data...")

# Start image processing thread
image_thread = threading.Thread(target=process_image)
image_thread.daemon = True
image_thread.start()

try:
    while True:
        time.sleep(1)  # Keep the main program running
except KeyboardInterrupt:
    print("Process interrupted by user.")
finally:
    client.loop_stop()
    client.disconnect()
    print("Disconnected from MQTT broker.")
