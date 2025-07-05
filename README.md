![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-blue?logo=esphome)
![License](https://img.shields.io/github/license/r0bb10/ESPHome-Olimpia-Bridge)
![GitHub stars](https://img.shields.io/github/stars/r0bb10/ESPHome-Olimpia-Bridge?style=social)

# Olimpia Bridge for ESPHome

**Olimpia Bridge** is a custom [ESPHome](https://esphome.io) component that enables full control of Olimpia Splendid fancoil HVAC units over **Modbus ASCII via UART**.

> This component is based on the excellent original project by [@dumpfheimer](https://github.com/dumpfheimer/olimpia_splendid_bi2_modbus_controller) — huge thanks for the hard work and reverse-engineering he did made the creation of this component possible!

This ESPHome version replicates the same communication behavior, supports multiple slave units, and integrates seamlessly with Home Assistant using its **native API**. Unlike the original project, it does **not use HTTP or MQTT**, resulting in tighter integration and simpler dynamic configuration with builtin OTA.

## 🚀 Features

- **Modbus ASCII communication** with Olimpia Splendid devices (like B0872 kit)
- Communication behavior and timing comply with official Olimpia Splendid specifications & recommendations
- Full **climate control** (mode, fan speed, target temperature) with state action
- **External ambient temperature** injection (via Home Assistant) with RAM and EEPROM fallback
- Persistent storage of climate state in flash memory with **Powerloss detection & recovery**
- **Robust boot fallback detection and self-healing**
- Optional **water temperature sensor** per unit
- **Support for multiple devices on the same UART bus**
- **Randomized polling & control scheduling** to prevent Modbus collisions
- Custom HA services:  
  - `olimpia_bridge.write_register` (allows writing to configuration registers for advanced tuning parameters)
  - `olimpia_bridge.read_register` (manual reads for debugging) 

## ⚙️ Architecture

- Centralized **Modbus ASCII handler** manages all communication over RS-485 using a finite state machine (FSM):
  - `IDLE → SEND_REQUEST → WAIT_RESPONSE → PROCESS_RESPONSE`
- Fully **asynchronous and non-blocking**, using a queued request system with per-request callbacks.
- Supports **multiple independent climate units** via per-device Modbus addresses.
- All communication is LRC-validated to ensure protocol integrity.
- Implements automatic **retry and timeout logic**, with detailed logging and recovery paths.
- Built-in support for **dynamic configuration**, sensor injection, and flash persistence.

## 🛡️ Why This Is a Good Design

- One central FSM ensures **strict Modbus timing and serialization**, avoiding collisions on the RS-485 bus.
- **Safe multidevice support**: each unit operates independently while sharing the communication backend.
- **Extensible and modular**: climate logic, sensors, and Modbus handling are clearly separated and easily maintained.
- **Reliable recovery**: devices restore control state after power loss using real device reads or flash fallback logic.
- Optimized for **ESPHome + Home Assistant**, providing full state sync, real-time control, and HA-native climate/fan modes.

## 🧩 Components

- **`ModbusAsciiHandler`**  
  Low-level Modbus ASCII engine that handles frame encoding/decoding, LRC validation, and manages the request queue via a finite state machine (FSM).

- **`OlimpiaBridge`**  
  High-level orchestrator that:
  - Manages UART and RE/DE pin control.
  - Registers the Modbus FSM handler.
  - Coordinates all connected `OlimpiaBridgeClimate` devices.
  - Exposes ESPHome services for direct register read/write.

- **`OlimpiaBridgeClimate`**  
  Represents a climate-controlled zone:
  - Handles state restoration, persistence, and HA UI synchronization.
  - Reads/writes registers 101/102/103 and interprets register 1 (water temp) and 9 (valve status).
  - Supports smart ambient injection via EMA and dynamic target/control logic.

## 🔍 How It Works

1. **Initialization**
   - Sets up UART communication and RE/DE GPIOs for RS-485 half-duplex.
   - Binds internal Modbus ASCII handler.
   - Registers all climate entities and loads saved control state from EEPROM if available.
   - Performs a boot-time sync by reading registers `101` (control state) and `102` (target temperature) to recover actual device state.
   - If a fallback (AUTO + 22°C) is detected, it restores and re-applies the last known state from flash.

2. **Periodic Control & Polling Loop**
   - Every 60 seconds (with jitter), each unit runs a **control cycle**:
     - `101` → Sends control flags (power, mode, fan) as a bitmapped register.
     - `102` → Sends target temperature (`°C × 10`).
     - `103` → Sends ambient room temperature (filtered EMA from HA).
     - Then re-reads `101` and `102` to verify and update current HA state.
   - Every 30 seconds, a **status poll** runs:
     - `9` → Reads valve status to infer HVAC action (heating/cooling/idle).
     - `1` → Reads water temperature (`°C × 10`) and publishes to optional sensor.

3. **Home Assistant Control**
   - Users can change mode, target temp, and fan speed via the standard HA climate entity.
   - Commands are persisted in flash and automatically re-applied after reboots or power loss.

4. **Ambient Temperature Injection & Persistence**
   - Home Assistant pushes ambient temperature via:
     ```yaml
     id(<unit>).set_external_ambient_temperature(x);
     ```
   - Values are smoothed using an **EMA filter** (`ema_alpha`, default `0.2`) with trend confirmation to reject noise.
   - EEPROM save logic prevents flash wear and stores a backup value:
     - On first valid HA update,
     - If temp changes significantly after 1h,
     - Or during fallback boot recovery.
   - If no updates are received for 15 minutes, the EMA resets.
   - On reboot, fallback value is restored and pushed to `103` until HA resumes.

## 🛠 Installation

The cleanest way and easiest way to keep your component up-to-date is to install it via GitHub directly.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/r0bb10/esphome-olimpia-bridge.git
      ref: main
    components: [olimpia_bridge]
```

## ⚙️ Example Configuration

```yaml
uart:
  id: modbus_uart
  tx_pin:
    number: GPIO37  # DI (TX line to RS-485)
    inverted: true
  rx_pin:
    number: GPIO39  # RO (RX line from RS-485)
    inverted: true
  baud_rate: 9600
  data_bits: 7
  stop_bits: 1
  parity: EVEN

olimpia_bridge:
  id: modbus_ascii_bridge
  uart_id: modbus_uart
  re_pin: GPIO35  # RE (Receive Enable)
  de_pin: GPIO33  # DE (Driver Enable)
  climates:
    - id: living_room
      name: Living Room Fancoil
      address: 1  # Modbus address (1–247)
      ema_alpha: 0.25  # Optional: EMA smoothing factor for ambient temp (default: 0.2)
      water_temperature_sensor:  # Optional: Sensor for water temp (register 1)
        name: Living Room Water Temperature
        unit_of_measurement: "°C"
        accuracy_decimals: 1
        device_class: temperature
        state_class: measurement

    - id: bedroom
      name: Bedroom Unit
      address: 2
      ema_alpha: 0.1  # More smoothing
      # No water_temperature_sensor defined for this unit
```

### 🔄 Ambient Temperature Injection

```yaml
sensor:
  - platform: homeassistant
    id: living_room_temp
    entity_id: sensor.living_room_temp
    on_value:
      then:
        - lambda: |-
            float rounded = roundf(x * 10.0f) / 10.0f;
            id(living_room_main).set_external_ambient_temperature(x);
            id(living_room_slave).set_external_ambient_temperature(x);

  - platform: homeassistant
    id: bedroom_temp
    entity_id: sensor.bedroom_temp
    on_value:
      then:
        - lambda: |-
            float rounded = roundf(x * 10.0f) / 10.0f;
            id(bedroom).set_external_ambient_temperature(x);
```

### 🏷️ Services

- **olimpia_bridge.write_register**
Write a register like programming a device address.

```yaml
- service: olimpia_bridge.write_register
  data:
    address: 1
    register: 200
    value: 5
```

- **olimpia_bridge.read_register**
Read a single register.

```yaml
- service: olimpia_bridge.read_register
  data:
    address: 1
    register: 103
```

## 📥 Input / Measured Registers

| Register | Description                                      | Access | Notes                                               |
|----------|--------------------------------------------------|--------|-----------------------------------------------------|
| 1        | Water temperature                                | R      | Reported by each slave, optional sensor             |
| 9        | Valve and system status bitfield                 | R      | Bit 13 = ev1, 14 = boiler, 15 = chiller             |
| 101      | HVAC status bits                                 | R/W    | Controls power, mode, and fan                       |
| 102      | Setpoint temperature (°C × 10)                   | R/W    | Desired setpoint temperature                        |
| 103      | External ambient temperature (°C × 10)           | R/W    | Written from External Home Assistant sensor         |


## 🧰 Persistent Configuration Registers

These registers should not be written frequently to avoid EEPROM wear. Only use them for configuration tasks.

| Register | Description                                      | Access | Notes                                               |
|----------|--------------------------------------------------|--------|-----------------------------------------------------|
| 200      | Modbus slave address                             | R/W    | Must be unique; avoid overwrites                    |
| 202      | Minimum allowed setpoint (°C)                    | R/W    | Default: 15°C                                       |
| 203      | Maximum allowed setpoint (°C)                    | R/W    | Default: 30°C                                       |
| 204      | Low Band Hysteresis (.°C)                        | R/W    | Default: 5                                          |
| 205      | High Band Hysteresis (.°C)                       | R/W    | Default: 10                                         |
| 210      | Min fan speed in cool mode (e.g. 680)            | R/W    | Unit: RPM                                           |
| 211      | Min fan speed in heat mode (e.g. 680)            | R/W    | Unit: RPM                                           |
| 212      | Max fan speed in cool mode (e.g. 950)            | R/W    | Unit: RPM                                           |
| 213      | Max fan speed in heat mode (e.g. 950)            | R/W    | Unit: RPM                                           |
| 214      | Max fan speed in min mode (e.g. 680)             | R/W    | Unit: RPM                                           |
| 215      | Max fan speed in night mode (e.g. 680)           | R/W    | Unit: RPM                                           |
| 216      | Fan speed with electrical heating (e.g. 1400)    | R/W    | Unit: RPM                                           |
| 217      | Minimum water temp for heating (°C)              | R/W    | Default: 30°C                                       |
| 218      | Minimum water temp for electrical heating (°C)   | R/W    | Default: 30°C (same as 217)                         |
| 219      | Maximum water temp for cooling (°C)              | R/W    | Default: 20°C                                       |
| 220      | Water temp alarm delay (min)                     | R/W    | Default: 5 min                                      |
| 233      | Current operating mode                           | R      | 3=heating, 5=cooling, 0=auto, 7=fan-only?           |


## ⚠️ Notes

- Ensure Olimpia Splendid units use Modbus ASCII and follow standard register map.
- The `101 → 102 → 103` sequence is written cyclically every 60s to meet controller expectations.
- Component ensures proper UART timing, buffer management, and frame parsing as per original controller.

## 🧪 Troubleshooting

- **No response from unit**:
  - Ensure correct UART settings: 9600 7E1 (7 data bits, even parity, 1 stop bit).
  - Verify RE/DE GPIO wiring matches Olimpia Splendid interface board.
  - Confirm device address matches expected Modbus slave address.

- **Ambient temperature never updates**:
  - Make sure your HA sensor `on_value` lambda is actually pushing data via `.set_external_ambient_temperature(x);`.
  - Enable debug logs in ESPHome (`logger: level: DEBUG`) to verify internal EMA and control logs.

- **Climate entity always shows "OFF"**:
  - Register `101` might be stuck in standby — check startup recovery logs.
  - Power-loss fallback may have triggered. Check if state was restored from flash.

## 📦 Compatibility

- Tested on:
  - ESP32 (ESP32-S3, ESP32-WROOM, LOLIN-S2-MINI)
  - ESP8266 (WEMOS D1 MINI)
  - Olimpia Splendid B0872 Modbus Interface
- Requires:
  - ESPHome ≥ 2023.8.0
  - Olimpia Splendid units using **Modbus ASCII protocol**

## 🤝 Contributing

Pull requests, bug reports, and ideas for improvement are always welcome!

Feel free to open an [issue](https://github.com/r0bb10/ESPHome-Olimpia-Bridge/issues) if:

- You want to suggest a feature
- You encounter an unsupported register
- You found a timing bug or Modbus-related edge case

## 📝 License

This project is licensed under the [MIT License](LICENSE).