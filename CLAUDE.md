# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 multi-device SPI communication system with four ESP32-S3 boards:
- **Master**: SPI master with WiFi Captive Portal and serial command interface
- **Slave 1**: SPI slave with NeoPixel LED + PCA9685 servo control (signature: 0xDEAD)
- **Slave 2**: SPI slave with NeoPixel RGB LED control (signature: 0xCAFE)
- **Slave 3**: SPI slave with NeoPixel RGB LED control (signature: 0xBEEF)

The master communicates with all slaves via SPI at 4 MHz and provides a Captive Portal web interface.

## Build Commands

```bash
# Build specific environment
pio run -e master
pio run -e slave1
pio run -e slave2
pio run -e slave3

# Build all (default: master, slave1, slave2)
pio run

# Upload to specific device
pio run -e master -t upload
pio run -e slave1 -t upload

# Monitor serial output
pio device monitor -e master

# Upload and monitor
pio run -e master -t upload && pio device monitor -e master

# Upload LittleFS filesystem (web interface)
pio run -e master -t uploadfs

# Clean build
pio run -t clean
```

## Hardware Configuration

### SPI Pins (all devices)
| Signal | Master GPIO | Slave GPIO |
|--------|-------------|------------|
| MOSI   | 35          | 35         |
| MISO   | 36          | 36         |
| SCLK   | 37          | 37         |
| CS S1  | 5           | 46         |
| CS S2  | 6           | 46         |
| CS S3  | 7           | 46         |

### Serial Ports
- Master: COM6, Slave 1: COM20, Slave 2: COM22, Slave 3: COM19 (115200 baud)

### Slave 1 Servo (PCA9685 via I2C)
- I2C: SDA=GPIO9, SCL=GPIO10, OE=GPIO11
- Channels: Servo1=CH3 (SG90), Servo2=CH7 (MG996R continuous), Servo3=CH13 (SG90)

## Architecture

### Master
- `main.cpp`: Entry point calling `WifiPortalSetup()`/`Loop()` and `MasterSpiSetup()`/`Loop()`
- `WifiPortal.h`: Captive Portal AP ("captive" @ 4.3.2.1), DNS server, ESPAsyncWebServer
- `MasterSpi.h`: SPI transfers, benchmarks, stress tests, serial command handler

### Slaves
All slaves use identical architecture:
- FreeRTOS task `spiReceiveTask` pinned to core 1 (priority 2)
- ESP-IDF SPI slave driver (SPI2_HOST) with DMA
- Non-blocking LED updates via flag (`led_update_pending`)
- 5-second throughput reporting

**Slave 1 additionally includes**:
- `Control_servo.h`: PCA9685 servo control with position/continuous modes
- SPI commands 0x10-0x13 for servo control via master

### SPI Protocol
- 4-byte transactions: `[command, param1, param2, param3]`
- Responses: `[sig_high, sig_low, counter_high, counter_low]`
- LED commands: 0x00=Off, 0x01=Red, 0x02=Green, 0x03=Blue
- Slave 1 servo: 0x10/0x11=set angle (normal/reverse), rx_buf[1]=servo_num, rx_buf[2]=angle

### Transfer Timing
- **Fast mode** (benchmarks): 1µs setup/hold, 5µs inter-transaction
- **Normal mode** (LED): 10µs setup/hold, 50µs inter-transaction
- **LED stress test**: 150µs inter-transaction

## Serial Commands

### Master Commands
```
LED:        s1r/s1g/s1b/s1o, s2r/s2g/s2b/s2o, s3r/s3g/s3b/s3o
Batch:      allr/allg/allb/allo
Benchmark:  b1/b2/b3 (1k iter), ball, bfast (10k iter)
Stress:     max [sec], led [sec]
```

### Slave 1 Servo Commands
```
s<n>:<angle>     - Set angle (s1:90, s2:45)
s<n>:<angle>r    - Reversed direction
s<n>:<angle>s    - Smooth movement
pwm<n>:<val>     - Raw PWM value
rot<n>:<deg>:<speed> - Continuous rotate (rot2:90:50)
test<n>, sweep<n>, status, stop
```

## Common Gotchas

1. **LittleFS**: Web interface requires separate `uploadfs` after code upload
2. **Captive Portal**: May need manual navigation to http://4.3.2.1
3. **Samsung devices**: Require IP in public space (hence 4.3.2.1 instead of 192.168.x.x)
4. **Slave signatures**: Verify 0xDEAD/0xCAFE/0xBEEF in stress test output for debugging
5. **Servo blocking**: `ServoMoveSmooth` and `ServoRotateByAngle` use blocking delays
6. **COM port conflicts**: Verify ports in platformio.ini match your system
