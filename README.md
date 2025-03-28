# QMI8658A Sensor Data Streaming via MQTT

## Overview
This project enables real-time streaming of 3-axis accelerometer and gyroscope data from the QMI8658A sensor using an ESP32-S3. The data is transmitted via MQTT to a Raspberry Pi running Mosquitto as the MQTT broker.

## Features
- Real-time sensor data acquisition from QMI8658A (Accelerometer & Gyroscope)
- MQTT-based data transmission
- ESP32-S3 as the publisher
- Raspberry Pi with Mosquitto as the MQTT broker
- Configurable data transmission rate
- Periodic CSV file storage and transmission (optional)

## Hardware Requirements
- ESP32-S3 development board
- QMI8658A IMU sensor
- Raspberry Pi (for MQTT broker)
- Wi-Fi network for communication

## Software Requirements
- ESP-IDF v5.3.2 (for ESP32-S3 development)
- Mosquitto MQTT broker on Raspberry Pi
- Python/MQTT client (for data reception & processing, optional)

## Installation & Setup

### ESP32-S3 Setup
1. Clone the repository:
   ```sh
   git clone <repo-url>
   cd <repo-name>
   ```
2. Set up the ESP-IDF environment (Ensure ESP-IDF v5.3.2 is installed and sourced).
3. Configure the project:
   ```sh
   idf.py menuconfig
   ```
   - Set Wi-Fi credentials
   - Configure MQTT broker details
4. Build and flash the firmware:
   ```sh
   idf.py flash monitor
   ```

### Raspberry Pi MQTT Broker Setup
1. Install Mosquitto:
   ```sh
   sudo apt update
   sudo apt install -y mosquitto mosquitto-clients
   ```
2. Enable and start Mosquitto service:
   ```sh
   sudo systemctl enable mosquitto
   sudo systemctl start mosquitto
   ```
3. (Optional) Configure Mosquitto authentication if needed.

### MQTT Subscriber Example (Python)
To receive data on a client device:
```python
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    print(f"Received message on {msg.topic}: {msg.payload.decode()}")

client = mqtt.Client()
client.on_message = on_message
client.connect("<raspberry-pi-ip>", 1883, 60)
client.subscribe("sensor/qmi8658a")
client.loop_forever()
```

## Usage
- Ensure the ESP32-S3 is powered and connected to Wi-Fi.
- Run the MQTT subscriber on the Raspberry Pi or another MQTT client.
- View real-time accelerometer and gyroscope data.

## Future Enhancements
- Implement data filtering and processing
- Improve error handling and reconnection strategies
- Enable encrypted MQTT communication

## License
This project is licensed under the MIT License.

## Acknowledgments
Special thanks to the developers of ESP-IDF and Mosquitto for their open-source contributions.

