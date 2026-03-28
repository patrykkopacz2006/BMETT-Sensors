# ESP32 Research Streaming Code

This repository contains the ESP32 firmware used to acquire and stream data from multiple sensors over BLE.

The code reads from:
- **2 TLx493D magnetic sensors**
- **1 BNO08x IMU**
- **2 analog muscle signal inputs**

The ESP32 processes the incoming sensor data, packages it into binary packets, and transmits it over Bluetooth Low Energy (BLE) to a connected device.

---

## Overview

This firmware is designed for real-time sensor streaming and uses separate sampling schedules for each sensor type:

- **Magnetometers:** 5000 µs sampling period
- **IMU:** 10000 µs sampling period
- **Muscle inputs:** 1000 µs sampling period

Data is grouped into packets before transmission to reduce BLE overhead.

---

## Features

- BLE-based wireless streaming
- Independent timing for each sensor type
- Low-pass filtering applied to magnetic and muscle data
- Step limiting applied to muscle readings to reduce sudden spikes
- Quaternion-to-Euler conversion for IMU orientation
- Binary packet structure for efficient transmission

---

## Hardware Used

This code is written for an ESP32-based setup using:

- **ESP32**
- **2x Infineon TLx493D magnetic sensors**
- **1x Adafruit BNO08x IMU**
- **2 analog muscle sensor channels**

---

## Pin Configuration

### I2C Buses
The code uses three separate I2C buses:

- **Magnetometer 1:** SDA = 8, SCL = 9
- **Magnetometer 2:** SDA = 17, SCL = 18
- **IMU:** SDA = 11, SCL = 12

### Analog Inputs
- **Muscle 1:** GPIO 4
- **Muscle 2:** GPIO 5

---

## BLE Information

The ESP32 advertises as:

- **Device Name:** `ESP32_RESEARCH_STREAM`

BLE UUIDs used:

- **Service UUID:** `91B10000-4E5A-4B1E-8F00-91B191B191B1`
- **Stream Characteristic UUID:** `91B10001-4E5A-4B1E-8F00-91B191B191B1`

The code sends notifications through the stream characteristic whenever a packet is ready.

---

## Packet Types

Three packet types are used:

- **PKT_MAG = 1** → magnetometer data
- **PKT_IMU = 2** → IMU orientation data
- **PKT_MUSCLE = 3** → muscle sensor data

Each packet contains:
- packet type
- sample count
- starting timestamp
- sampling interval
- packed sensor samples

---

## Sampling and Packet Sizes

### Magnetometer
- Sampling interval: **5000 µs**
- Samples per packet: **4**

### IMU
- Sampling interval: **10000 µs**
- Samples per packet: **2**

### Muscle
- Sampling interval: **1000 µs**
- Samples per packet: **20**

---

## Signal Processing

### Magnetometer Data
Magnetometer readings are low-pass filtered using a smoothing factor of **0.12** before being packetized.

### IMU Data
The BNO08x provides quaternion rotation data, which is converted into:
- yaw
- pitch
- roll

These values are stored in centidegrees before transmission.

### Muscle Data
Muscle readings are:
1. read from analog pins
2. step-limited to reduce abrupt jumps
3. low-pass filtered using a smoothing factor of **0.08**

---

## Required Libraries

To compile this code in the Arduino IDE, install the required libraries for the sensors and BLE support.

From the code, the following libraries are required:

- `Wire`
- `TLx493D_inc.hpp`
- `Adafruit_BNO08x`
- `BLEDevice`
- `BLEServer`
- `BLEUtils`
- `BLE2902`

You may need to install the relevant vendor libraries through the Arduino Library Manager or from the manufacturer’s GitHub repositories depending on how your environment is set up.

---

## How to Run

1. Open the code in **Arduino IDE**
2. Install all required libraries
3. Select the correct **ESP32 board**
4. Select the correct **COM port**
5. Upload the code to the ESP32
6. Open your BLE client or host application and connect to:
   - `ESP32_RESEARCH_STREAM`

---

## Notes

- The code uses `analogReadResolution(12)`, so analog readings are expected at 12-bit resolution.
- If the IMU resets during runtime, the code re-enables the IMU report automatically.
- BLE advertising restarts automatically after disconnection.
- Data is transmitted only while a BLE device is connected.

---

## File Purpose

This firmware is intended for real-time research data acquisition and wireless streaming from multiple sensor sources using an ESP32.
