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

// --- Fan speed levels (PRG bits 0–2) ---
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

  st.on = (reg & (1 << 7)) == 0;            // Bit 7 = STBY (0 = ON)
  st.cp = (reg & (1 << 12)) != 0;           // Bit 12 = CP
  uint8_t mode = (reg >> 13) & 0x03;        // Bits 13–14 = Mode
  uint8_t fan = reg & 0x07;                 // Bits 0–2 = Fan speed

  switch (mode) {
    case 0b00: st.mode = Mode::AUTO; break;
    case 0b01: st.mode = Mode::HEATING; break;
    case 0b10: st.mode = Mode::COOLING; break;
    default:   st.mode = Mode::UNKNOWN; break;
  }

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
  // Core component methods
  void setup() override;
  void loop() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // Configuration
  void set_address(uint8_t address) { this->address_ = address; }
  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }
  void set_water_temp_sensor(sensor::Sensor *sensor) { this->water_temp_sensor_ = sensor; }
  void set_external_ambient_temperature(float temp);
  void set_ema_alpha(float alpha) { this->ambient_ema_alpha_ = alpha; }

  // State control and polling
  void status_poll_cycle();
  void control_cycle();
  void apply_last_known_state();
  void maybe_save_state();
  void restore_or_refresh_state();
  void update_climate_action_from_valve_status();
  void read_water_temperature();

  // Register management
  uint16_t build_command_register(bool on, Mode mode, FanSpeed fan_speed);
  void write_control_registers_cycle(std::function<void()> callback = nullptr);

 protected:
  void update_state_from_parsed(const ParsedState &parsed);
  uint16_t get_status_register();

  // Modbus configuration
  uint8_t address_;
  ModbusAsciiHandler *handler_{nullptr};

  // Sensors
  sensor::Sensor *water_temp_sensor_{nullptr};

  // Climate state
  float target_temperature_{22.0f};
  float current_temperature_{NAN};
  float external_ambient_temperature_{NAN};
  float smoothed_ambient_{NAN};
  float ambient_ema_alpha_{0.2f};
  bool first_ha_ambient_received_{false};
  uint32_t last_external_temp_update_{0};

  // Device power and control state
  bool on_{false};
  bool boot_cycle_done_{false};
  bool boot_recovery_in_progress_{false};
  bool boot_recovery_done_{false};
  bool block_control_until_recovery_{true};
  Mode mode_{Mode::UNKNOWN};
  FanSpeed fan_speed_{FanSpeed::UNKNOWN};

  // Persistence
  ESPPreferenceObject pref_;
  ESPPreferenceObject saved_state_pref_;
  SavedState last_saved_state_{};

  // External Temperature Management
  bool using_fallback_external_temp_{false};
  bool has_received_external_temp_{false};
  bool external_temp_received_from_ha_{false};
  bool reg103_read_from_device_{false};  // verify
  uint32_t last_external_temp_flash_write_{0};

  // Timing
  uint32_t system_boot_time_ms_{0};
  uint32_t last_valve_status_poll_{0};
  uint32_t last_water_temp_poll_{0};
  uint32_t last_update_time_{0};
  uint32_t next_control_cycle_ms_{0};
  uint32_t next_status_poll_ms_{0};
};

}  // namespace olimpia_bridge
}  // namespace esphome
