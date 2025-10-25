# RAK4631 Multi-Sensor LoRaWAN Node

A simplified and reliable LoRaWAN sensor node implementation for the RAKwireless WisBlock RAK4631 that transmits environmental and GPS data every 5 minutes.

## Overview

This project implements a battery-efficient sensor node that reads data from environmental sensors (BME680) and GPS (u-blox GNSS), then transmits the data via LoRaWAN. The device uses unconfirmed messages for reliable transmission without duty cycle issues.

## Hardware Requirements

- **RAKwireless WisBlock RAK4631** (nRF52840 + SX1262)
- **Adafruit BME680** Environmental Sensor (Temperature, Humidity, Pressure, Gas)
- **SparkFun u-blox GNSS** Module (GPS)
- **WisBlock Base Board** (for sensor connections)

### Pin Configuration

- `WB_IO2` - Sensor power control pin (powers sensors on/off between readings)
- I2C - Used for BME680 and GNSS communication

## Features

- **Environmental Monitoring**: Temperature (°F), Humidity (%), Pressure (hPa), Gas Resistance (Ohms)
- **GPS Tracking**: Latitude, Longitude, Altitude
- **Air Quality Classification**: Based on gas resistance readings (GREAT/BETTER/OK/POOR/BAD)
- **LoRaWAN Connectivity**: OTAA join, unconfirmed messages for reliable transmission
- **Power Management**: Sensors powered down between readings to conserve battery
- **Automatic Transmission**: Sends data every 5 minutes (configurable)

## Software Requirements

### Arduino IDE Setup

1. Install Arduino IDE (version 1.8.x or 2.x)
2. Add RAKwireless board support:
   - Go to **File > Preferences**
   - Add this URL to "Additional Boards Manager URLs":
     ```
     https://raw.githubusercontent.com/RAKwireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_index.json
     ```
   - Go to **Tools > Board > Boards Manager**
   - Search for "RAKwireless" and install the package

### Required Libraries

Install these libraries via **Tools > Manage Libraries**:

- **SX126x-Arduino** (v2.0.32 or later) - LoRaWAN communication
- **Adafruit Unified Sensor** (v1.1.15 or later)
- **Adafruit BME680 Library** (v2.0.5 or later)
- **Adafruit BusIO** (v1.17.4 or later)
- **SparkFun u-blox GNSS Arduino Library** (v2.2.28 or later)
- **CayenneLPP** (v1.6.1 or later) - Payload encoding
- **ArduinoJson** (v7.4.2 or later)

Libraries included with RAKwireless BSP:
- Wire (I2C communication)
- SPI
- Adafruit TinyUSB Library

## Configuration

### LoRaWAN Credentials

Before uploading, update the following values in the sketch with your device credentials from your LoRaWAN network server (TTN, Chirpstack, etc.):

```cpp
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeDeviceEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x00, 0x00, 0x00, 0x00 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
```

### Transmission Interval

Adjust the transmission interval (default is 5 minutes):

```cpp
#define LORAWAN_APP_INTERVAL 300000  // milliseconds (300000 = 5 minutes)
```

### LoRaWAN Region

The sketch is configured for US915. To change regions, modify:

```cpp
lmh_init(&lora_callbacks, lora_param_init, doOTAA, CLASS_A, LORAMAC_REGION_US915);
```

Supported regions: `LORAMAC_REGION_US915`, `LORAMAC_REGION_EU868`, `LORAMAC_REGION_AU915`, etc.

## Installation

1. Clone or download this repository
2. Open `RAK4361_gps-env-1.3.1.ino` in Arduino IDE
3. Update LoRaWAN credentials (AppEUI, DeviceEUI, AppKey)
4. Select board: **Tools > Board > RAKwireless nRF Boards > WisCore RAK4631 Board**
5. Select port: **Tools > Port > [Your COM Port]**
6. Click **Upload**

## Serial Monitor Output

Open the Serial Monitor at **115200 baud** to view device status:

```
=== RAK4631 START ===
Init BME680...
BME680 OK
Init GNSS...
GNSS OK
Init LoRa...
LoRa OK
=== SETUP DONE ===
Transmit interval: 300 seconds
Join OK!

=== CYCLE START ===
Uptime: 305 seconds
===== Environmental Data =====
Temperature: 73.4F
Humidity: 49.3%
Pressure: 1003 hPa
Gas: 2094 Ohms
Air Quality: BAD
==============================
Getting GPS...
GPS FIX OK
Sending 26 bytes...
TX OK - Packet queued for transmission
=== CYCLE END ===
```

## Data Payload Format

Data is transmitted using CayenneLPP format with the following channels:

| Channel | Type | Data | Unit |
|---------|------|------|------|
| 1 | Temperature | BME680 Temperature | °C |
| 2 | Relative Humidity | BME680 Humidity | % |
| 3 | Barometric Pressure | BME680 Pressure | hPa |
| 4 | Analog Input | BME680 Gas Resistance | Ohms |
| 5 | GPS | Latitude, Longitude, Altitude | degrees, meters |

**Note:** GPS data (channel 5) is only included when a valid fix is obtained.

## Decoding Payloads

### The Things Network (TTN)

TTN automatically decodes CayenneLPP payloads. Enable CayenneLPP formatter in your application settings.

### Manual Decoding

Use the CayenneLPP decoder library or refer to the [CayenneLPP specification](https://developers.mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload).

## Troubleshooting

### Device won't join network
- Verify LoRaWAN credentials are correct
- Check that your gateway is online and in range
- Ensure the correct region is selected
- Verify your device is registered on the network server

### TX FAIL errors
- If you see "TX FAIL", the radio may be busy or in duty cycle restriction
- The sketch uses unconfirmed messages to minimize this issue
- Ensure `LORAWAN_DUTYCYCLE_OFF` is set (already configured)

### No GPS fix
- Allow 30+ seconds for initial GPS lock (cold start)
- Ensure clear view of the sky
- GPS may not work indoors
- If GPS fails, environmental data is still transmitted

### BME680 not found
- Check I2C connections
- Verify BME680 address (0x76 or 0x77)
- The sketch will continue without BME680, sending dummy values

## Power Consumption

The device uses power management to extend battery life:
- Sensors are powered via `WB_IO2` pin
- Sensors are powered down between readings
- GPS acquisition timeout: 30 seconds
- Typical sleep current: ~15-20µA (depends on configuration)

## Known Issues

- GPS cold start can take 30+ seconds
- Indoor GPS acquisition may fail completely
- First transmission after join may occasionally fail (retries automatically)

## Future Improvements

- [ ] Add deep sleep mode for even lower power consumption
- [ ] Implement adaptive data rate (ADR) optimization
- [ ] Add local data logging to SD card
- [ ] Battery voltage monitoring
- [ ] Configurable GPS timeout based on fix success rate

## License

This project is provided as-is for educational and development purposes.

## Acknowledgments

- RAKwireless for the excellent WisBlock ecosystem
- Adafruit for BME680 sensor libraries
- SparkFun for u-blox GNSS library
- The Things Network community for LoRaWAN support

## Contributing

Feel free to submit issues and pull requests for improvements!

## Version History

- **v1.3.1** - Initial release with reliable 5-minute transmission cycle
  - Unconfirmed messages for duty cycle compliance
  - Environmental and GPS data transmission
  - Power management implementation

---

**Author**: Shawn Falconbury  
**Last Updated**: October 25, 2025  
**Hardware**: RAKwireless WisBlock RAK4631 with GPS and ENV 
**Network**: LoRaWAN (US915)
