# ESP32 SIM7600E Data Logger Documentation

## 1. Overview

This document provides documentation for the `ESP32_SIM7600E_DataLogger.ino` firmware. The project is designed for a data collection and logging task utilizing an ESP32 microcontroller.

The system interfaces with a Battery Management System (BMS) through an RS-485 transceiver connected to one of the ESP32's UART ports. It collects various battery parameters from the BMS, gathers GPS and board-level data, and uploads the aggregated information to a Supabase server. The firmware also hosts a web server for real-time data viewing and Over-the-Air (OTA) firmware updates.

## 2. Data Collection Control

The firmware allows for granular control over which data points are collected. This is managed through pre-processor `#define` directives located at the top of the `.ino` file. Users can enable or disable specific data collection modules by changing the value from `1` (enabled) to `0` (disabled).

### 2.1. BMS Data Commands

You can selectively request different data packets from the BMS. This is useful for optimizing the data collection process and reducing RS-485 bus traffic.

**Toggles:**
- `ENABLE_BMS_BASIC_STATUS`: Cumulative voltage, current, SOC.
- `ENABLE_BMS_VOLTAGE_LIMITS`: Max/Min cell voltage.
- `ENABLE_BMS_TEMPERATURE`: Max/Min temperature.
- `ENABLE_BMS_MOSFET_STATUS`: Charge/discharge MOS status, cycle life.
- `ENABLE_BMS_SYSTEM_INFO`: Number of cells, temperature sensors.
- `ENABLE_BMS_CELL_VOLTAGES`: Individual cell voltages.
- `ENABLE_BMS_CELL_TEMPERATURES`: Individual cell temperatures.
- `ENABLE_BMS_CELL_BALANCE`: Cell balance states.
- `ENABLE_BMS_FAULT_STATUS`: Fault codes.

### 2.2. GPS, GSM, and Board Data

Similarly, data from the integrated GPS/GSM module and the ESP32 board itself can be toggled.

**Toggles:**
- `ENABLE_GPS_DATA`: Collects location, speed, and time data.
- `ENABLE_GSM_DATA`: Collects network signal strength, operator info, etc.
- `ENABLE_BOARD_DATA`: Collects ESP32 stats like CPU frequency, heap size, etc.

## 3. BMS RS-485 Packet Parsing

A key aspect of the communication protocol with the BMS is understanding how the response packet is structured. When the ESP32 sends a command to the BMS over the RS-485 bus, the transceiver line first echoes the sent command back to the ESP32's receiver pin.

This echoed command consists of **13 bytes**. The firmware is designed to discard these initial 13 bytes of any incoming packet. The actual data payload sent back by the BMS begins from the **14th byte (index 13) onwards**. The parsing logic exclusively processes this part of the stream to extract meaningful battery data.

## 4. Pin Configuration

The firmware is configured for a specific hardware setup. The pin mappings are defined as follows:

- **Serial Monitor (UART0):**
  - **TX:** `GPIO1`
  - **RX:** `GPIO3`
  - *Note: These are the default pins for the `Serial` object.*

- **SIM7600E Module (UART2 for GPS/GSM):**
  - **TX:** `GPIO27`
  - **RX:** `GPIO26`
  - *Note: The GPS module is integrated into the SIM7600E and shares this UART port.*

- **BMS Communication (RS-485):**
  - **DI (Data Input):** `GPIO33` (ESP32 TX)
  - **RO (Receiver Output):** `GPIO32` (ESP32 RX)
  - **DE/RE (Driver/Receiver Enable):** `GPIO25`
  - *Note: During BMS data collection, the UART2 pins are temporarily re-assigned to communicate with the RS-485 transceiver.*

## 5. Prerequisites and Operational Logic

### 5.1. WiFi Credentials

Before uploading the firmware, you **must** configure your WiFi credentials. Locate the following lines in the code and replace the placeholders with your network's SSID and password.

```cpp
const char* ssid = "your_ssid";
const char* password = "your_password";
```

### 5.2. GPS Fix Requirement

The system has a critical dependency on a valid GPS signal. **No data is collected or sent to the Supabase server until the GPS module achieves a successful fix.** The device will wait for a fix before proceeding with the main data logging loop. This ensures that all logged data is accurately geotagged.

### 5.3. Data Destination: Supabase

Once a GPS fix is obtained and data is collected, the firmware uploads the data payload in JSON format to a pre-configured Supabase project. The Supabase URL and authentication key must be set in the firmware:

```cpp
const char* supabase_url = "https://....supabase.co";
const char* supabase_key = "your_supabase_anon_key";
```

## 6. Accessing Data and Updates

### 6.1. Serial Monitor

The simplest way to monitor the data being collected is through the Arduino IDE's Serial Monitor or any other serial terminal connected to the ESP32's UART0 port at a baud rate of **115200**. The collected data is printed in a formatted JSON structure.

### 6.2. Web Server

The ESP32 hosts a web server on the local WiFi network. You can find the IP address of the device in the serial monitor upon startup.

- **Real-time Data:** To view the latest collected data packet, navigate to:
  `http://<ESP32_IP_ADDRESS>/`

- **Firmware (OTA) Updates:** The firmware leverages the `ElegantOTA` library to allow for wireless updates. To access the update page, navigate to:
  `http://<ESP32_IP_ADDRESS>/update`

  From this page, you can upload the new compiled `.bin` file to update the firmware without a physical connection.
