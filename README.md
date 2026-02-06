# 🚀 STM32F407 FOTA Update System using ESP32 + SPI + AWS S3

## 📌 Overview

This project implements a complete **Firmware Over-The-Air (FOTA)** update system for the STM32F407 microcontroller using an ESP32 as a WiFi OTA bridge. Firmware is downloaded from a cloud server and transferred to STM32 via SPI using a chunk-based protocol. A custom smart bootloader supports dual-bank (ping-pong) firmware updates with metadata validation and safe bank switching.

The system is designed for reliable remote firmware upgrades without physical access to the device.

---

## 🧩 System Components

- **STM32F407**
  - Smart OTA bootloader
  - Dual application banks (Bank A / Bank B)
  - Metadata sector for active bank tracking
  - SPI slave OTA receiver
  - Non-blocking OTA state machine

- **ESP32**
  - WiFi connection
  - OTA bridge controller
  - Downloads firmware from server
  - Sends firmware to STM32 via SPI
  - Controls STM32 reset line

- **Python Flask Server**
  - Serves firmware binaries
  - Provides version endpoint
  - Auto-syncs firmware from AWS S3 bucket

---

## ✨ Features

- Dual-bank firmware architecture
- Smart self-healing bootloader
- Metadata-based bank selection
- Safe ping-pong update strategy
- SPI chunk-based firmware transfer
- Non-blocking OTA state machine
- Automatic bank detection using program counter
- Flash sector erase + byte programming
- ESP32 version tracking using Preferences
- AWS S3 firmware hosting
- Flask server auto-update sync
- LED status indicators for debug

---

## 🧠 Flash Memory Layout (STM32F407)

```
0x08000000 — Bootloader
0x08020000 — Application Bank A (Sector 5)
0x08060000 — Application Bank B (Sector 7)
0x080E0000 — Metadata Sector (Sector 11)
```

---

## 📂 Project Structure

```
bootloader.c      -> Smart STM32 OTA bootloader
slot_a.c          -> OTA application build for Bank A
slot_b.c          -> OTA application build for Bank B
esp32.ino         -> ESP32 OTA bridge firmware
server.py         -> Flask OTA server with S3 sync

firmware/
  slot_a.bin
  slot_b.bin
  version.txt
```

---

## 🔄 OTA Update Flow

1. ESP32 connects to WiFi
2. ESP32 reads current firmware version (stored internally)
3. ESP32 requests `/version` from server
4. If newer version exists:
   - ESP32 resets STM32
   - Gets active bank ID via SPI
   - Requests opposite-bank firmware
5. STM32 erases inactive bank
6. Firmware sent in SPI chunks
7. STM32 writes firmware to flash
8. Metadata updated with new bank ID
9. ESP32 reboots STM32
10. Bootloader jumps to new firmware

---

## 🔌 Hardware Connections

| ESP32 Pin | STM32 Pin |
|-----------|------------|
GPIO18 | PA5 (SPI SCK) |
GPIO23 | PA7 (SPI MOSI) |
GPIO19 | PA6 (SPI MISO) |
GPIO5  | PA4 (CS) |
GPIO21 | NRST |
GND    | GND |

---

## 💡 LED Status Indication

### Bootloader
- Red ON → bootloader running
- Green → jumping Bank A
- Blue → jumping Bank B
- Fast red blink → error (no valid app)

### Application
- Bank A → Green blink
- Bank B → Orange blink
- OTA transfer → Blue activity blink

---

## 🛠 Setup Instructions

### Build STM32

- Build bootloader and flash at `0x08000000`
- Build Slot A firmware → flash at `0x08020000`
- Build Slot B firmware → flash at `0x08060000`
- Use separate linker scripts for each bank

---

### Run OTA Server

```bash
pip install flask requests
python server.py
```

Server runs on:

```
http://<PC-IP>:5000
```

---

### Configure ESP32

Edit in ESP32 code:

```
WIFI_SSID
WIFI_PASS
SERVER_URL
```

Upload using Arduino IDE.

---

### Upload Firmware to S3

Upload these files to your S3 bucket:

```
slot_a.bin
slot_b.bin
version.txt
```

Increase version.txt to trigger update.

---

## 🔐 Safety Mechanisms

- Stack pointer validation before jump
- Metadata magic word check
- Self-initializing metadata sector
- Never overwrite running firmware
- Flash error flag clearing
- Sector erase before program
- Chunk acknowledgment protocol
- Automatic fallback to valid bank

---

## 📈 Future Improvements

- CRC firmware verification
- Secure firmware signing
- HTTPS download support
- Delta updates
- Resume interrupted OTA
- Encrypted transfer

---
