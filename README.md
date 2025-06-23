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
- Optional **water temperature sensor** per unit 
- Custom HA services:  
  - `olimpia_bridge.write_register` (allows writing to configuration registers for advanced tuning parameters)
  - `olimpia_bridge.read_register` (manual reads for debugging) 

## ⚙️ Architecture

- Integrated **finite state machine (FSM)** to serialize Modbus communication (`IDLE → SEND_REQUEST → WAIT_RESPONSE → PROCESS_RESPONSE`)
- **Async, non-blocking** requests via a queue with per-request callbacks to avoid collision and cluttering
- Validates all Modbus frames via LRC checksum
- Handles multiple devices on the same bus (independent Modbus addresses <- fully programmable)
- Implements retry and error logic with logging

## 🛡️ Why This Is a Good Design

- Centralized control over Modbus bus management (e.g., UART pin setup, FSM registration)
- Prevents multiple handlers or UART conflicts
- Ensures all climate nodes use the same FSM queue and obey Modbus timing/sequencing rules
- Designed for reliability, clarity, and integration with Home Assistant via ESPHome

## 🧩 Components

- `ModbusAsciiHandler`: low-level Modbus ASCII encoder/decoder + FSM
- `OlimpiaBridge`: actual orchestrator to handle multiple slaves, connects FSM to HA services
- `OlimpiaBridgeClimate`: climate device abstraction with persistence and UI sync

## 🔍 How It Works

1. **Initialization (setup)**  
   - Configure UART and RE/DE pins for RS-485 direction control

2. **Polling & Control Loop (update)**  
   Every `update_interval` (default 60 s), for each configured climate unit:  
   1. **Write Register 101**  
      - Encodes **fan speed** (bits 0–2), **mode** (bits 2–3), and **standby** (bit 7)  
   2. **Write Register 102**  
      - Sends the **target temperature** (°C × 10)  
   3. **Write Register 103**  
      - Sends the **external ambient temperature** (°C × 10)  
      - Falls back to a previous value from RAM if received or fallbacks to EEPROM-persisted temp with autoupdate  
   4. **Read Register 1**  
      - Indipendently retrieves the **water temperature** (°C × 10) and publishes it to a sensor  

4. **Ambient-Temp Injection & Persistence**  
   - **Home Assistant sensor** pushes room temperature via a `lambda` calling  
     ```yaml
     id(<unit>).set_external_ambient_temperature(x);
     ```  
   - On first HA update, that value is used immediately and then saved to EEPROM once per 24 h  
   - On reboot or HA sensor loss, the last-received value in RAM is used else the last-saved EEPROM temperature is retrieved to continue to write the cycle.  

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
    number: GPIO37 # DI
    inverted: true
  rx_pin:
    number: GPIO39 # RO
    inverted: true
  baud_rate: 9600
  data_bits: 7
  stop_bits: 1
  parity: EVEN

olimpia_bridge:
  id: modbus_ascii_bridge
  uart_id: modbus_uart
  re_pin: GPIO35 # RE
  de_pin: GPIO33 # DE
  climates:
    - id: living_room
      name: Living Room Fancoil
      address: 1 # Modbus address
      water_temperature_sensor: # optional
        name: Living Room Water Temp
    - id: bedroom
      name: Bedroom Unit
      address: 2 # Modbus address
      # no water sensor
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
            id(living_room).set_external_ambient_temperature(x);

  - platform: homeassistant
    id: bedroom_temp
    entity_id: sensor.bedroom_temp
    on_value:
      then:
        - lambda: |-
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

---

## ⚠️ Notes

- Ensure Olimpia Splendid units use Modbus ASCII and follow standard register map.
- The `101 → 102 → 103` sequence is written cyclically every 60s to meet controller expectations.
- Component ensures proper UART timing, buffer management, and frame parsing as per original controller.
