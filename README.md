# Worker Monitor

An ESP32-based worker condition monitoring system that collects various biometric and environmental data through sensors.

## Overview

This project combines multiple sensors with an ESP32 microcontroller to create a comprehensive worker condition monitoring system. It tracks vital signs and environmental conditions, providing real-time data for worker safety applications.

## Hardware Requirements

- ESP32 Development Board
- MPU6500 Motion Sensor (accelerometer, gyroscope, magnetometer)
- BMP280 Pressure/Temperature Sensor
- MAX30102 Pulse Oximeter and Heart Rate Sensor
- DHT Temperature and Humidity Sensor

## Libraries

- MPU9250_WE
- Adafruit Unified Sensor
- Adafruit BMP280 Library
- SparkFun MAX3010x Pulse and Proximity Sensor Library
- DHT sensor library
- Arduino_JSON

## Instructions

### Connections

- MPU6500: Connect SDA to GPIO21, SCL to GPIO22
- BMP280: Connect SDA to GPIO21, SCL to GPIO22
- MAX30102: Connect SDA to GPIO21, SCL to GPIO22
- DHT sensor: Connect data pin to GPIO16
- Buzzer: Connect VCC pin to D14

### Setup

1. Install PlatformIO IDE.
2. Clone this repository:
   ```
   git clone https://github.com/he1senbrg/worker-monitor
   ```

## Configuration

1. Update the WiFi credentials in the code.
2. Configure any server connection settings if needed.

## Data

- Motion data (acceleration, gyroscope, orientation) via MPU6500
- Atmospheric pressure and temperature via BMP280
- Heart rate and blood oxygen levels via MAX30102
- Ambient temperature and humidity via DHT sensor

## License

This project is licensed under the [MIT License](LICENSE).
