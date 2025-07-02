#pragma once

#include <cmath>  // Required for NAN
#include "esphome/core/preferences.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "modbus_ascii_handler.h"

namespace esphome {
namespace olimpia_bridge {

// --- Operating modes (EI field in register 101) ---
enum class Mode : uint8_t {
  AUTO     = 0,
  COOLING  = 1,
  HEATING  = 2,
  UNKNOWN  = 0xFF
};

// --- Fan speed levels per PRG field (bits 0–2) ---
enum class FanSpeed : uint8_t {
  AUTO    = 0b000,
  MIN     = 0b001,
  NIGHT   = 0b010,
  MAX     = 0b011,
  UNKNOWN = 0xFF
};

// --- Parsed register 101 state ---
struct ParsedState {
  bool on = false;
  bool cp = false;
  FanSpeed fan_speed = FanSpeed::UNKNOWN;
  Mode mode = Mode::UNKNOWN;
};

// --- Persisted state structure ---
struct SavedState {
  bool on;
  Mode mode;
  FanSpeed fan_speed;
  float target_temperature;
  climate::ClimateAction last_action;
};

// --- Utility to decode register 101 ---
inline ParsedState parse_command_register(uint16_t reg) {
  ParsedState st;

  // Bit 7 = STBY (0 = ON, 1 = OFF) → ON if bit 7 is 0
  st.on = (reg & (1 << 7)) == 0;

  // Bit 12 = CP
  st.cp = (reg & (1 << 12)) != 0;

  // Bits 13–14 = Mode
  uint8_t mode = (reg >> 13) & 0x03;
  switch (mode) {
    case 0b00: st.mode = Mode::AUTO; break;
    case 0b01: st.mode = Mode::HEATING; break;
    case 0b10: st.mode = Mode::COOLING; break;
    default:   st.mode = Mode::UNKNOWN; break;
  }

  // Bits 0–2 = Fan speed
  uint8_t fan = reg & 0x07;
  switch (fan) {
    case 0x00: st.fan_speed = FanSpeed::AUTO; break;
    case 0x01: st.fan_speed = FanSpeed::MIN; break;
    case 0x02: st.fan_speed = FanSpeed::NIGHT; break;
    case 0x03: st.fan_speed = FanSpeed::MAX; break;
    default:   st.fan_speed = FanSpeed::UNKNOWN; break;
  }

  return st;
}

// --- OlimpiaBridgeClimate class ---
class OlimpiaBridgeClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void loop() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }
  void set_external_ambient_temperature(float temp);
  void set_ema_alpha(float alpha) { this->ambient_ema_alpha_ = alpha; }

  void status_poll_cycle();  // Periodically polls register 9
  void update_climate_action_from_valve_status();
  void maybe_save_state();  // safe, minimal flash writes
  void restore_or_refresh_state();
  void control_cycle();
  void read_water_temperature();
  void apply_last_known_state();

  uint16_t build_command_register(bool on, Mode mode, FanSpeed fan_speed);

 protected:
  void update_state_from_parsed(const ParsedState &parsed);
  void write_control_registers_cycle(std::function<void()> callback = nullptr);
  uint16_t get_status_register();
  uint32_t last_valve_status_poll_{0};  // Tracks last poll time of ev1
  uint32_t last_water_temp_poll_{0};  // Tracks last poll time of reg1

  // Modbus config
  uint8_t address_;
  ModbusAsciiHandler *handler_{nullptr};

  // Sensors
  sensor::Sensor *water_temp_sensor_{nullptr};

  // Climate state
  float target_temperature_{22.0f};
  float current_temperature_{NAN};
  float external_ambient_temperature_{NAN};

  // EMA smoothing for ambient temperature
  float smoothed_ambient_{NAN};            // Smoothed version of ambient temp
  float ambient_ema_alpha_ = 0.2f;         // EMA smoothing factor (0.0 - 1.0)

  bool on_{false};
  bool boot_cycle_done_{false};
  bool boot_recovery_done_{false};               // True after boot state recovery completes
  bool block_control_until_recovery_{true};      // Prevent user commands until recovery is done
  Mode mode_{Mode::UNKNOWN};
  FanSpeed fan_speed_{FanSpeed::UNKNOWN};

  // State machine & persistence
  uint32_t last_update_time_{0};
  ESPPreferenceObject pref_;  // External temp fallback
  ESPPreferenceObject saved_state_pref_;  // Climate state
  SavedState last_saved_state_{};

  bool using_fallback_external_temp_{false};
  bool has_received_external_temp_{false};
  bool external_temp_received_from_ha_{false};
  bool reg103_read_from_device_{false};
  uint32_t last_external_temp_flash_write_{0};

  // System booted mark
  uint32_t system_boot_time_ms_{0};

  // Randomized per-device scheduling
  uint32_t next_control_cycle_ms_{0};
  uint32_t next_status_poll_ms_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
