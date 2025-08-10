![ESPHome](https://img.shields.io/badge/ESPHome-Compatible-blue?logo=esphome)
![License](https://img.shields.io/github/license/r0bb10/ESPHome-Olimpia-Bridge)
![GitHub stars](https://img.shields.io/github/stars/r0bb10/ESPHome-Olimpia-Bridge?style=social)

# Olimpia Bridge for ESPHome

Turn your Olimpia Splendid HVAC into a smart climate system with ESPHome and Home Assistant. Control temperature, fan speeds, and operating modes directly from your smart home dashboard.

> Based on [@dumpfheimer](https://github.com/dumpfheimer/olimpia_splendid_bi2_modbus_controller)'s work, who not only reverse-engineered the protocol but also created the first standalone controller with Home Assistant integration. Thank you!

## 🚀 Features

- **Modbus ASCII communication** with Olimpia Splendid devices allowing full **climate control** 
- Communication behavior and timing comply with official Olimpia Splendid specifications & recommendations
- **Smart temperature handling**:
  - **External temperature injection** from Home Assistant with RAM/EEPROM fallback
  - **Advanced EMA filtering** for ultra-smooth and stable temperature readings:
    - Configurable per-device smoothing factors
    - Smart noise rejection via trend validation
  - **Temperature management features**:
    - Temperature change trend analysis for a more accurate zone monitoring
    - Rapid response to significant changes
    - Fallback temperature recovery and inactivity reset routine
- **Intelligent powerloss protection**:
  - Advanced fallback detection and self-healing
  - Automatic state recovery from flash
  - Real-time state verification and correction
- **Robust state management**:
  - Real device state sync on startup
  - Command verification and retry
  - Persistent storage with wear leveling
- **Virtual presets**: Supports "Auto" and "Manual" presets for enhanced climate control via automation
- **Flexible configuration**:
  - Optional **water temperature sensor** monitoring per unit
  - Per-device EMA settings
  - Global EMA enable/disable switch

## ⚙️ Architecture

The component is structured in three main layers, each handling distinct responsibilities:

### Communication Layer
- **`ModbusAsciiHandler`**: The foundation layer implementing the Modbus ASCII protocol
  - Centralized handler with finite state machine (FSM): `IDLE → SEND_REQUEST → WAIT_RESPONSE → PROCESS_RESPONSE`
  - Responsible for all hardware configuration and initialization (UART, RE/DE pins)
  - All communication is LRC-validated with automatic frame verification
  - Hardware-optimized timing for RS-485 half-duplex operation
  - Implements robust request queuing (FIFO), fully asynchronous and non-blocking
  - Per-request callback system for flexible response processing
  - Provides automatic retry on communication errors

### Orchestration Layer
- **`OlimpiaBridge`**: The central coordinator
  - Implements the shared Modbus backend for all climate units
  - Manages device addressing and climate entity coordination
  - Provides direct register access services for configuration

### Climate Control Layer
- **`OlimpiaBridgeClimate`**: Individual zone controller
  - Implements the ESPHome climate interface
  - Manages zone-specific state, configuration, and persistence
  - Handles temperature processing and EMA filtering
  - Controls operation modes and presets
  - Performs state recovery and flash wear leveling for its own zone

1. **Data Flow**
   - External temperature readings from Home Assistant
   - EMA filtering with configurable alpha
   - Trend validation and noise rejection
   - Automatic calibration after inactivity

2. **State Persistence**
   - Flash storage with wear leveling
   - Recovery points for power loss
   - Fallback values for safety

## 🛡️ Why This Is a Good Design

Imagine your home's climate control as a well-orchestrated system where multiple air conditioning units work together seamlessly. This design makes that possible while solving several real-world challenges that users face with smart HVAC systems.

The architecture is built to be rock-solid reliable. Just like a good autopilot system, there's a central "brain" that carefully manages all communication with your AC units. This prevents them from talking over each other and ensures every command gets through correctly. If you have multiple units, they'll all work together harmoniously without interfering with each other - like having a skilled conductor leading an orchestra.

The system proves incredibly resilient to the kinds of problems that plague smart home devices. Power outages? No problem. Network hiccups? Covered. When the power comes back on, your units won't reset to factory settings - they'll remember exactly how you like them and restore those settings automatically. 

The temperature handling system shows particular intelligence. Instead of reacting to every tiny temperature fluctuation (which can make your AC work harder than necessary), it uses sophisticated filtering to ensure smooth, comfortable operation. This is similar to how a car's cruise control smoothly maintains speed without constant acceleration and braking.

The Home Assistant integration demonstrates thoughtful design. The complex inner workings are hidden behind a clean, user-friendly interface. Users get all the benefits of a sophisticated system while keeping the simplicity of controlling their AC units through familiar Home Assistant controls.

For power users, the component includes virtual "Auto" and "Manual" presets that open up interesting automation possibilities. For example, you could create your own Home Assistant automations that use the "Auto" preset as a signal to run your custom climate control logic - perhaps adjusting settings based on time of day, occupancy, or even energy prices. The "Manual" preset could then serve as a temporary override switch: when a room is switched to "Manual", your automations would know to skip that zone while continuing to manage others. When switched back to "Auto", the zone would rejoin your automation scheme. It's like providing the building blocks for a smart override system - how you implement it is up to your imagination and automation skills.

---
*This analysis has been written by GitHub Copilot after reviewing the complete codebase, including component architecture, internal logic, and actual implementation. The analogies and explanations are based on real, working code patterns identified in the project.*

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
esphome:
  name: "${hostname}"
  comment: "Olimpia Splendid Bridge"
  devices:
    - id: olimpia_living
      name: "Living Room Unit"
    - id: olimpia_bedroom
      name: "Bedroom Unit"

api:
  services:
    - service: read_register
      variables:
        address: int
        reg: int
      then:
        - lambda: |-
            id(modbus_ascii_bridge).read_register(address, reg);
    - service: write_register
      variables:
        address: int
        reg: int
        value: int
      then:
        - lambda: |-
            id(modbus_ascii_bridge).write_register(address, reg, value);
    - service: dump_configuration
      variables:
        address: int
      then:
        - lambda: |-
            id(modbus_ascii_bridge).dump_configuration(address);

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
  # 1. Single EN pin
  en_pin: GPIO32  # EN (Direction control, HIGH=TX, LOW=RX)
  # 2. Dual-pin
  # re_pin: GPIO35  # RE (Receive Enable)
  # de_pin: GPIO33  # DE (Driver Enable)
  error_ratio_sensor:
    name: Modbus Error Ratio
  use_ema: false  # Optional: enable/disable EMA filtering (default: true)
  climates:
    - name: Living Room Unit
      id: living_room
      device_id: olimpia_living
      address: 1  # Modbus address
      ema_alpha: 0.25  # Optional: EMA smoothing factor for ambient temp (default: 0.2)
      water_temperature_sensor:  # Optional: sensor for water temp
        name: Living Room Water Temperature
        device_id: olimpia_living
      device_error_ratio_sensor:  # Optional: per-device error ratio sensor
        name: Living Room Error Ratio
        device_id: olimpia_living
      presets_enabled: true  # Optional: exposes virtual presets to HA (default: false)
      disable_mode_auto: true  # Optional: hide AUTO mode from HA (default: false)
      disable_fan_quiet: true  # Optional: hide QUIET fan mode from HA (default: false)
      min_temperature: 16.0  # Optional (default: 15.0) - Register 202 value needs to be updated
      max_temperature: 28.0  # Optional (default: 30.0) - Register 203 value needs to be updated
      target_temperature_step: 0.1  # Optional (default: 0.5)

    - name: Bedroom Unit
      id: bedroom
      device_id: olimpia_bedroom
      address: 2
      ema_alpha: 0.1  # More smoothing
      water_temperature_sensor:  # Optional: sensor for water temp
        name: Bedroom Water Temperature
        device_id: olimpia_bedroom
      device_error_ratio_sensor:  # Optional: per-device error ratio sensor
        name: Bedroom Error Ratio
        device_id: olimpia_bedroom
      presets_enabled: false
      disable_mode_auto: false
      disable_fan_quiet: false

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

- **olimpia_bridge.dump_configuration**
Dump all configuration registers for a device.

```yaml
- service: olimpia_bridge.dump_configuration
  data:
    address: 1
```

For detailed information about available registers and their functions, please see [DEVELOPMENT.md](DEVELOPMENT.md#-core-operating-registers).


## 📦 Compatibility

- Tested on:
  - ESP32 (ESP32-S3, ESP32-WROOM, LOLIN-S2-MINI, WT32-ETH01)
  - ESP8266 (WEMOS D1 MINI)
- Requires:
  - Olimpia Splendid units using **Modbus ASCII protocol** (B0872 Modbus Interface)

## 📝 License

This project is licensed under the [MIT License](LICENSE).