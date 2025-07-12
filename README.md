# ESP32-MQTT: Remote Control and Monitoring via MQTT and Qt

This project is part of the final assignment for the **Microcontroller Intensification** course in the Master’s Degree in Electronic Systems for Intelligent Environments (MSEEI) at the University of Málaga.

It presents a complete solution for **bidirectional interaction between a PC and an ESP32 WROVER-KIT board**, using the **MQTT protocol** and **JSON-formatted messages**. The microcontroller firmware is built on **FreeRTOS**, and the PC interface is implemented with **Qt**.

## 📡 System Architecture

The system consists of:

- A **Qt-based graphical interface (PC)** with user controls.
- An **embedded application running on the ESP32**, structured with FreeRTOS.
- An **MQTT broker** for communication (can be local, e.g., Mosquitto, or public).
- **MQTT topics-based communication**, supporting multi-client synchronization.

---

## ✅ Implemented Features

### 🧩 Basic Functionality

1. **Ping**: Verifies connectivity between PC and ESP32 via a response message.
2. **RGB LED control**: Toggle Red, Green, and Blue channels in GPIO mode.
3. **Button state reading**:
   - Polling mode via a UI button.
   - Asynchronous notification mode via MQTT interrupts.
4. **LED mode switch (GPIO / PWM)**:
   - Brightness control through PWM sliders.
   - Smooth transition between modes.
5. **ADC reading (potentiometer)**:
   - Sampled periodically every 200 ms.
   - Real-time visualization on the Qt interface.
6. **Multi-client synchronization**:
   - All Qt instances reflect consistent LED and button states.
7. **Connectivity monitoring**:
   - Uses MQTT `retained` messages and `last will` topic to detect ESP32 status.
8. **Async mode for button state reporting**:
   - Sends MQTT message immediately upon button press using interrupts.

### 🧪 Advanced Features

9. **Remote temperature sensor (DS1621) reading**:
   - Start/stop control from Qt.
   - Configurable reading interval.
   - MQTT message publishing and LCD display.
10. **Controllable ADC data streaming**:
    - Red LED ON while active.
    - Blue LED toggles on each transmission.
11. **BLE scan support**:
    - Initiated/stopped from Qt.
    - Device list (MAC, RSSI, name) published via MQTT.
    - Green LED indicator on LCD when active.

---

## 🧰 Technologies Used

| Component           | Tool / Framework               |
|---------------------|--------------------------------|
| Microcontroller     | ESP32 WROVER-KIT               |
| Firmware            | ESP-IDF + FreeRTOS             |
| Communication       | MQTT                           |
| GUI Application     | Qt 5.x / Qt 6.x                |
| Message Format      | JSON (via `frozen` library)    |
| LCD & Peripherals   | LVGL, GPIO, ADC, I2C, BLE      |

---

## 🚀 Getting Started

### 1. Requirements

- MQTT broker (`mosquitto` recommended)
- PC with Qt Creator installed
- ESP-IDF (ESP32 development framework)
- Shared WiFi network for PC and ESP32

### 2. Execution Steps

1. **Build and flash** the ESP32 firmware.
2. **Run the MQTT broker** (e.g., `mosquitto -v`).
3. **Launch the Qt GUI** from Qt Creator.
4. **Interact** with the GUI and observe ESP32 responses.

---

## 🧪 Project Status

- [x] All basic requirements implemented and verified.
- [x] Full multi-client sync and MQTT connectivity feedback.
- [x] Advanced features (temperature sensor, BLE) integrated.
- [x] JSON messaging format and robust MQTT handling implemented.

---

## 🎓 Acknowledgements

This project was developed for academic purposes within the Master’s Degree in Electronic Systems for Intelligent Environments (MSEEI) at the **University of Málaga**, under the supervision of:

- José Manuel Cano  
- Eva González  
- Ignacio Herrero  
