#include "olimpia_bridge_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "Climate";

// --- Helper functions for logging ---
static const char *mode_to_string(Mode mode) {
  switch (mode) {
    case Mode::AUTO: return "AUTO";
    case Mode::COOLING: return "COOL";
    case Mode::HEATING: return "HEAT";
    case Mode::UNKNOWN: default: return "UNKNOWN";
  }
}

static const char *fan_speed_to_string(FanSpeed fan) {
  switch (fan) {
    case FanSpeed::AUTO: return "AUTO";
    case FanSpeed::MIN: return "LOW";
    case FanSpeed::NIGHT: return "QUIET";
    case FanSpeed::MAX: return "HIGH";
    case FanSpeed::UNKNOWN: default: return "UNKNOWN";
  }
}

// --- Register 101 encoder (bit-mapped control register) ---
uint16_t OlimpiaBridgeClimate::build_command_register(bool on, Mode mode, FanSpeed fan_speed) {
  uint16_t reg = 0;

  // --- Bits 0–2: PRG fan speed ---
  // 000 = Auto, 001 = Min, 010 = Night, 011 = Max
  reg |= static_cast<uint8_t>(fan_speed) & 0x07;

  // --- Bit 7: STBY (working condition) ---
  // 0 = Activated, 1 = Standby (OFF)
  if (!on)
    reg |= (1 << 7);

  // --- Bit 12: CP presence contact ---
  reg &= ~(1 << 12);  // Ensure CP is always 0

  // --- Bits 13–14: EI mode of functioning ---
  // 10 = Cooling, 01 = Heating, 00 = Auto
  switch (mode) {
    case Mode::AUTO:
      reg |= (0b00 << 13);
      break;
    case Mode::HEATING:
      reg |= (0b01 << 13);
      break;
    case Mode::COOLING:
      reg |= (0b10 << 13);
      break;
    default:
      reg |= (0b00 << 13);  // Fallback to AUTO
      break;
  }
  return reg;
}

// --- Setup ---
void OlimpiaBridgeClimate::setup() {
  // --- Prepare preference entries
  this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash() ^ 0x1030U);
  this->saved_state_pref_ = global_preferences->make_preference<SavedState>(this->get_object_id_hash() ^ 0x2040U);

  // --- Load fallback external temp from flash
  float fallback = NAN;
  bool has_fallback = this->pref_.load(&fallback);
  if (has_fallback) {
    this->external_ambient_temperature_ = fallback;
    this->using_fallback_external_temp_ = true;
    ESP_LOGD(TAG, "[%s] Climate setup → fallback_used=true temp=%.2f°C", this->get_name().c_str(), fallback);
  } else {
    ESP_LOGD(TAG, "[%s] Climate setup → fallback_used=false (no flash value)", this->get_name().c_str());
  }

  // --- Load or initialize climate control state
  SavedState recovered;
  bool found = this->saved_state_pref_.load(&recovered);
  if (found) {
    this->on_ = recovered.on;
    this->mode_ = recovered.mode;
    this->fan_speed_ = recovered.fan_speed;
    this->target_temperature_ = recovered.target_temperature;
    this->last_saved_state_ = recovered;

    ESP_LOGI(TAG, "[%s] Recovered last saved user state → mode=%d fan=%d on=%d target=%.1f°C",
             this->get_name().c_str(), static_cast<int>(this->mode_), static_cast<int>(this->fan_speed_),
             this->on_, this->target_temperature_);
  } else {
    // --- First boot or flash reset: use defaults ---
    this->on_ = false;
    this->mode_ = Mode::AUTO;  // Represent OFF via on_=false + AUTO
    this->fan_speed_ = FanSpeed::AUTO;
    this->target_temperature_ = 22.0f;

    ESP_LOGW(TAG, "[%s] No saved user state found, applying default climate state (OFF, 22°C, FAN AUTO)", this->get_name().c_str());

    SavedState default_state{
      .on = this->on_,
      .mode = this->mode_,
      .fan_speed = this->fan_speed_,
      .target_temperature = this->target_temperature_,
    };
    this->last_saved_state_ = default_state;
    this->saved_state_pref_.save(&default_state);
  }

  // --- Sync fan_mode for HA UI
  switch (this->fan_speed_) {
    case FanSpeed::AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case FanSpeed::MIN:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case FanSpeed::NIGHT:
      this->fan_mode = climate::CLIMATE_FAN_QUIET;
      break;
    case FanSpeed::MAX:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  this->target_temperature = this->target_temperature_;  // Sync for HA UI
  this->publish_state();  // Ensure HA reflects the internal values immediately

  // --- Read reg103 from Olimpia device ---
  if (this->handler_ != nullptr) {
    this->handler_->read_register(this->address_, 103, 1,
      [this, has_fallback, fallback](bool success, const std::vector<uint16_t> &data) {
        if (success && !data.empty()) {
          float reg103 = data[0] * 0.1f;
          this->external_ambient_temperature_ = reg103;
          this->reg103_read_from_device_ = true;
          ESP_LOGD(TAG, "[%s] Read 103 → external ambient temperature: %.2f°C", this->get_name().c_str(), reg103);
          this->publish_state();
        } else if (has_fallback && !this->external_temp_received_from_ha_) {
          uint16_t reg = static_cast<uint16_t>(fallback * 10);
          this->handler_->write_register(this->address_, 103, reg,
            [this, fallback](bool success, const std::vector<uint16_t> &) {
              if (success) {
                ESP_LOGI(TAG, "[%s] Pushed fallback external temp %.1f°C to register 103 on boot", this->get_name().c_str(), fallback);
              } else {
                ESP_LOGW(TAG, "[%s] Failed to write fallback external temp to register 103", this->get_name().c_str());
              }
            });
        }
      });
  }

  // --- Initial read of water temperature at boot
  this->read_water_temperature();
  this->last_water_temp_poll_ = millis();  // Reset the polling timer

  // Kick off 101/102 read sequence for power-loss detection and actual device state
  this->restore_or_refresh_state();
}

// --- Traits ---
climate::ClimateTraits OlimpiaBridgeClimate::traits() {
  climate::ClimateTraits traits;
  traits.set_supports_current_temperature(true);
  traits.set_supports_action(true);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_AUTO,
  });
  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_QUIET,
    climate::CLIMATE_FAN_HIGH,
  });
  traits.set_visual_current_temperature_step(0.1);
  traits.set_visual_target_temperature_step(0.5);
  return traits;
}

// --- HA control callback ---
void OlimpiaBridgeClimate::control(const climate::ClimateCall &call) {
  bool state_changed = false;

  // Handle target temperature change
  if (call.get_target_temperature().has_value()) {
    this->target_temperature_ = *call.get_target_temperature();
    ESP_LOGI(TAG, "[%s] Target temperature set to %.1f°C", this->get_name().c_str(), this->target_temperature_);
    state_changed = true;
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    switch (new_mode) {
      case climate::CLIMATE_MODE_OFF:
        this->on_ = false;
        this->mode_ = Mode::AUTO;  // OFF = on_=false + AUTO
        break;
      case climate::CLIMATE_MODE_COOL:
        this->on_ = true;
        this->mode_ = Mode::COOLING;
        break;
      case climate::CLIMATE_MODE_HEAT:
        this->on_ = true;
        this->mode_ = Mode::HEATING;
        break;
      case climate::CLIMATE_MODE_AUTO:
        this->on_ = true;
        this->mode_ = Mode::AUTO;
        break;
      default:
        break;
    }
    state_changed = true;
  }

  // Handle fan mode change
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    if (new_fan == climate::CLIMATE_FAN_AUTO) this->fan_speed_ = FanSpeed::AUTO;
    else if (new_fan == climate::CLIMATE_FAN_LOW) this->fan_speed_ = FanSpeed::MIN;
    else if (new_fan == climate::CLIMATE_FAN_QUIET) this->fan_speed_ = FanSpeed::NIGHT;
    else if (new_fan == climate::CLIMATE_FAN_HIGH) this->fan_speed_ = FanSpeed::MAX;
    state_changed = true;
  }

  // If any setting changed, apply and persist
  if (state_changed) {
    this->write_control_registers_cycle([this]() {
      // Immediately check for action status after user control
      this->update_climate_action_from_valve_status();
    });

    SavedState current{
      .on = this->on_,
      .mode = this->mode_,
      .fan_speed = this->fan_speed_,
      .target_temperature = this->target_temperature_,
      .last_action = this->action
    };

    // Only save to flash if something actually changed
    if (memcmp(&this->last_saved_state_, &current, sizeof(SavedState)) != 0) {
      this->last_saved_state_ = current;
      if (this->saved_state_pref_.save(&current)) {
        ESP_LOGD(TAG, "[%s] Updated user state saved to flash: mode=%d fan=%d on=%d target=%.1f°C",
                 this->get_name().c_str(), static_cast<int>(this->mode_), static_cast<int>(this->fan_speed_),
                 this->on_, this->target_temperature_);
      } else {
        ESP_LOGW(TAG, "[%s] Failed to save state to flash", this->get_name().c_str());
      }
    } else {
      ESP_LOGD(TAG, "[%s] State unchanged, skipping flash write", this->get_name().c_str());
    }
  }

  // Refresh state and publish it to UI
  this->restore_or_refresh_state();
}

// --- Register 1 (water temp) ---
void OlimpiaBridgeClimate::read_water_temperature() {
  if (this->handler_ == nullptr || this->water_temp_sensor_ == nullptr) return;

  this->handler_->read_register(this->address_, 1, 1, [this](bool success, const std::vector<uint16_t> &data) {
    if (!success || data.empty()) {
      ESP_LOGW(TAG, "[%s] Failed to read water temperature", this->get_name().c_str());
      return;
    }

    float temp = data[0] * 0.1f;
    ESP_LOGI(TAG, "[%s] Water temperature: %.1f°C", this->get_name().c_str(), temp);
    this->water_temp_sensor_->publish_state(temp);
  });
}

// --- Periodic FSM control cycle ---
void OlimpiaBridgeClimate::control_cycle() {
  const uint32_t now = millis();

  // --- Skip control cycle if boot recovery is still in progress ---
  if (!this->boot_recovery_done_) {
    ESP_LOGD(TAG, "[%s] Skipping control cycle: boot recovery not complete", this->get_name().c_str());
    return;
  }  

  // Update every 60s or on first boot
  if (!this->boot_cycle_done_ || (now - this->last_update_time_ >= 60000)) {
    ESP_LOGD(TAG, "[%s] Starting control cycle", this->get_name().c_str());

    // --- Push control values to registers 101/102 ---
    this->write_control_registers_cycle();

    // --- Refresh current state from registers 101/102
    //  ---
    this->restore_or_refresh_state();

    // --- Always push last known external ambient temp to register 103 ---
    if (!std::isnan(this->external_ambient_temperature_)) {
      uint16_t reg103 = static_cast<uint16_t>(this->external_ambient_temperature_ * 10);
      this->handler_->write_register(this->address_, 103, reg103, [this](bool success, const std::vector<uint16_t> &) {
        if (success) {
          ESP_LOGD(TAG, "[%s] Refreshed register 103 with external temp: %.1f°C", this->get_name().c_str(), this->external_ambient_temperature_);
        } else {
          ESP_LOGW(TAG, "[%s] Failed to refresh register 103 (external temp)", this->get_name().c_str());
        }
      });
    }

    // --- Update timestamps ---
    this->last_update_time_ = now;
    this->boot_cycle_done_ = true;
  }
}

// --- FSM write 101 → 102 → 103 ---
void OlimpiaBridgeClimate::write_control_registers_cycle(std::function<void()> callback) {
  if (this->handler_ == nullptr) return;

  uint16_t reg101 = this->get_status_register();
  uint16_t reg102 = static_cast<uint16_t>(this->target_temperature_ * 10);
  uint16_t reg103 = std::isnan(this->external_ambient_temperature_) ? 0 : static_cast<uint16_t>(this->external_ambient_temperature_ * 10);

  ESP_LOGI(TAG, "[%s] Writing control → Power: %s | Mode: %s | Fan: %s | Target: %.1f°C | Ambient: %.1f°C",
          this->get_name().c_str(),
          this->on_ ? "ON" : "OFF",
          mode_to_string(this->mode_),
          fan_speed_to_string(this->fan_speed_),
          this->target_temperature_,
          this->external_ambient_temperature_);

  this->handler_->write_register(this->address_, 101, reg101, [this, reg102, reg103, callback](bool ok1, const std::vector<uint16_t> &) {
    if (!ok1) {
      ESP_LOGW(TAG, "[%s] Failed to write register 101", this->get_name().c_str());
      return;
    }

    this->handler_->write_register(this->address_, 102, reg102, [this, reg103, callback](bool ok2, const std::vector<uint16_t> &) {
      if (!ok2) {
        ESP_LOGW(TAG, "[%s] Failed to write register 102", this->get_name().c_str());
        return;
      }

      this->handler_->write_register(this->address_, 103, reg103, [this, callback](bool ok3, const std::vector<uint16_t> &) {
        if (!ok3) {
          ESP_LOGW(TAG, "[%s] Failed to write register 103", this->get_name().c_str());
               return;
              }

              // Delay slightly before checking valve status to allow hardware to react
              if (callback) {
                this->set_timeout("valve_status_check", 500, [callback]() {
                  callback();
                });
              }
            });
        });
    });
}

// --- Update from parsed state (register 101) ---
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {
  // --- Copy state from parsed register 101
  this->on_ = parsed.on;
  this->mode_ = parsed.mode;
  this->fan_speed_ = parsed.fan_speed;

  // --- Map internal state to HA climate mode
  if (!this->on_) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (this->mode_) {
      case Mode::HEATING:
        this->mode = climate::CLIMATE_MODE_HEAT;
        break;
      case Mode::COOLING:
        this->mode = climate::CLIMATE_MODE_COOL;
        break;
      case Mode::AUTO:
        this->mode = climate::CLIMATE_MODE_AUTO;
        break;
      default:
        this->mode = climate::CLIMATE_MODE_OFF;
        break;
    }
  }

  // --- Map fan speed to HA fan mode
  switch (this->fan_speed_) {
    case FanSpeed::AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case FanSpeed::MIN:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case FanSpeed::NIGHT:
      this->fan_mode = climate::CLIMATE_FAN_QUIET;
      break;
    case FanSpeed::MAX:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      this->fan_mode.reset();
      break;
  }

  // --- Push temperatures to ESPHome climate state
  this->current_temperature = this->external_ambient_temperature_;
  this->target_temperature = this->target_temperature_;  // use internal as reference for now

  // --- Log state for debugging
  ESP_LOGD(TAG, "[%s] Updated state from reg101: ON=%d MODE=%d FAN=%d → current=%.1f°C target=%.1f°C",
           this->get_name().c_str(), this->on_, static_cast<int>(this->mode_),
           static_cast<int>(this->fan_speed_), this->current_temperature, this->target_temperature);

  // --- Update action if requested
  this->update_climate_action_from_valve_status();

  // --- Push state to HA
  this->publish_state();
}

// --- Compose Register 101 from Internal State ---
uint16_t OlimpiaBridgeClimate::get_status_register() {
  return this->build_command_register(this->on_, this->mode_, this->fan_speed_);
}

// --- External ambient temperature input (used for register 103) ---
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  ESP_LOGD(TAG, "[%s] set_external_ambient_temperature() called: %.2f°C | fallback=%s | received=%s",
           this->get_name().c_str(),
           temp,
           this->using_fallback_external_temp_ ? "true" : "false",
           this->has_received_external_temp_ ? "true" : "false");

  if (std::isnan(temp)) return;

  const uint32_t now = millis();
  const uint32_t BOOT_GRACE_PERIOD_MS = 2000;
  const uint32_t DEBOUNCE_TIME_MS = 30000;

  bool first_time = !this->has_received_external_temp_;
  bool refresh_flash = (now - this->last_external_temp_flash_write_ > 86400000UL);
  bool during_boot = (now < BOOT_GRACE_PERIOD_MS);

  // --- Debounce logic (per-instance, not static) ---
  if (std::isnan(this->debounce_candidate_temp_) || temp != this->debounce_candidate_temp_) {
    this->debounce_candidate_temp_ = temp;
    this->debounce_first_seen_ms_ = now;
    ESP_LOGD(TAG, "[%s] Debounce started for %.2f°C", this->get_name().c_str(), temp);
    return;
  }

  if (now - this->debounce_first_seen_ms_ < DEBOUNCE_TIME_MS) {
    ESP_LOGD(TAG, "[%s] Waiting for %.2f°C to stabilize (%.1f/%.1f sec)",
             this->get_name().c_str(), temp,
             (now - this->debounce_first_seen_ms_) / 1000.0f, DEBOUNCE_TIME_MS / 1000.0f);
    return;
  }

  ESP_LOGI(TAG, "[%s] External ambient temp confirmed: %.2f°C after %.1f sec",
           this->get_name().c_str(), temp, DEBOUNCE_TIME_MS / 1000.0f);

  // --- Update RAM ---
  this->external_ambient_temperature_ = temp;
  this->current_temperature = temp;
  this->has_received_external_temp_ = true;
  this->external_temp_received_from_ha_ = true;

  // --- Flash persistence logic ---
  if (!during_boot && (first_time || this->using_fallback_external_temp_ || refresh_flash)) {
    this->pref_.save(&temp);
    this->last_external_temp_flash_write_ = now;

    if (first_time)
      ESP_LOGI(TAG, "[%s] Stored first valid external temp to flash (initial persistence)", this->get_name().c_str());
    else if (this->using_fallback_external_temp_) {
      this->using_fallback_external_temp_ = false;
      ESP_LOGI(TAG, "[%s] Stored fresh external temp to flash (replacing fallback)", this->get_name().c_str());
    } else {
      ESP_LOGD(TAG, "[%s] Refreshed external temp in flash after 24h", this->get_name().c_str());
    }
  } else if (during_boot) {
    ESP_LOGD(TAG, "[%s] Skipping flash write during boot grace period", this->get_name().c_str());
  }

  // --- Publish updated climate state to Home Assistant ---
  this->publish_state();
}

// --- Restore Saved State from Flash ---
void OlimpiaBridgeClimate::apply_last_known_state() {
  SavedState recovered{};
  if (this->saved_state_pref_.load(&recovered)) {
    this->last_saved_state_ = recovered;
    ESP_LOGI(TAG, "[%s] Recovered state from flash: on=%d mode=%d fan=%d target=%.1f",
             this->get_name().c_str(),
             recovered.on, recovered.mode, recovered.fan_speed, recovered.target_temperature);
  } else {
    ESP_LOGW(TAG, "[%s] No saved state found in flash, skipping recovery", this->get_name().c_str());
    return;
  }

  this->on_ = recovered.on;
  this->mode_ = recovered.mode;
  this->fan_speed_ = recovered.fan_speed;
  this->target_temperature_ = recovered.target_temperature;
  this->target_temperature = this->target_temperature_;  // Sync to UI
  this->action = recovered.last_action;

  // --- Sync fan mode to UI ---
  switch (this->fan_speed_) {
    case FanSpeed::AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case FanSpeed::MIN:
      this->fan_mode = climate::CLIMATE_FAN_LOW;
      break;
    case FanSpeed::NIGHT:
      this->fan_mode = climate::CLIMATE_FAN_QUIET;
      break;
    case FanSpeed::MAX:
      this->fan_mode = climate::CLIMATE_FAN_HIGH;
      break;
    default:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
  }

  // --- Sync mode to UI ---
  switch (this->mode_) {
    case Mode::AUTO:
      this->mode = climate::CLIMATE_MODE_AUTO;
      break;
    case Mode::COOLING:
      this->mode = climate::CLIMATE_MODE_COOL;
      break;
    case Mode::HEATING:
      this->mode = climate::CLIMATE_MODE_HEAT;
      break;
    default:
      this->mode = climate::CLIMATE_MODE_AUTO;
      break;
  }
  if (!this->on_) {
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  this->publish_state();
}

// --- Boot State Recovery or Manual Refresh ---
void OlimpiaBridgeClimate::restore_or_refresh_state() {
  if (this->handler_ == nullptr) return;

  const bool is_first_boot = !this->boot_recovery_done_;

  if (is_first_boot)
    ESP_LOGI(TAG, "[%s] Boot state recovery: reading 101 + 102...", this->get_name().c_str());
  else
    ESP_LOGD(TAG, "[%s] Refreshing state from device...", this->get_name().c_str());

  this->handler_->read_register(this->address_, 101, 1,
    [this, is_first_boot](bool ok101, const std::vector<uint16_t> &data101) {
      if (!ok101 || data101.empty()) {
        ESP_LOGW(TAG, "[%s] Failed to read register 101", this->get_name().c_str());
        return;
      }

      uint16_t reg101 = data101[0];
      ParsedState parsed = parse_command_register(reg101);

      ESP_LOGD(TAG, "[%s] Read 101: 0x%04X → ON=%d MODE=%d FAN=%d", this->get_name().c_str(),
               reg101, parsed.on, parsed.mode, static_cast<int>(parsed.fan_speed));

      this->handler_->read_register(this->address_, 102, 1,
        [this, parsed, is_first_boot](bool ok102, const std::vector<uint16_t> &data102) {
          if (!ok102 || data102.empty()) {
            ESP_LOGW(TAG, "[%s] Failed to read register 102", this->get_name().c_str());
            return;
          }

          float target = data102[0] * 0.1f;
          ESP_LOGD(TAG, "[%s] Read 102 → target temperature: %.1f°C", this->get_name().c_str(), target);

          // --- POWER-LOSS RECOVERY ---
          if (is_first_boot && parsed.mode == Mode::AUTO && std::abs(target - 22.0f) < 0.2f) {
            ESP_LOGW(TAG, "[%s] Detected fallback state (AUTO + 22.0°C), restoring from saved flash state", this->get_name().c_str());
            this->apply_last_known_state();
            this->write_control_registers_cycle();  // Push corrected state back to device

            // Mark recovery as done, even in fallback case
            this->boot_recovery_done_ = true;
            this->block_control_until_recovery_ = false;
            ESP_LOGI(TAG, "[%s] Boot recovery fallback applied. Enabling control.", this->get_name().c_str());
            return;
          }

          // Set target temperature from register 102
          this->target_temperature_ = target;
          this->target_temperature = target;

          // Update internal + publish to HA
          this->update_state_from_parsed(parsed);

          ESP_LOGD(TAG, "[%s] Updated state → ON=%d MODE=%d FAN=%d target=%.1f°C",
                   this->get_name().c_str(), this->on_, static_cast<int>(this->mode_),
                   static_cast<int>(this->fan_speed_), this->target_temperature_);

          if (is_first_boot) {
            this->boot_recovery_done_ = true;
            this->block_control_until_recovery_ = false;
            ESP_LOGI(TAG, "[%s] Boot register read complete. Enabling control.", this->get_name().c_str());
          }
        });
    });
}

// --- Valve Status → Climate Action Mapping ---
void OlimpiaBridgeClimate::update_climate_action_from_valve_status() {
  if (this->handler_ == nullptr) return;

  this->handler_->read_register(this->address_, 9, 1,
    [this](bool success, const std::vector<uint16_t> &data) {
      if (!success || data.empty()) {
        ESP_LOGW(TAG, "[%s] Failed to read register 9 (valve status)", this->get_name().c_str());
        return;
      }

      const uint16_t reg9 = data[0];
      const uint8_t high_byte = (reg9 >> 8) & 0xFF;

      // Bit meaning based on reverse-engineered protocol
      bool ev1     = (high_byte & 0b01000000) != 0;  // Bit 6
      bool boiler  = (high_byte & 0b00100000) != 0;  // Bit 5
      bool chiller = (high_byte & 0b00010000) != 0;  // Bit 4

      climate::ClimateAction new_action = climate::CLIMATE_ACTION_OFF;

      // Ensure OFF action is only used when truly OFF
      if (!this->on_) {
        new_action = climate::CLIMATE_ACTION_OFF;
      } else if (ev1) {
        // Determine based on current mode
        if (this->mode_ == Mode::COOLING) {
          new_action = climate::CLIMATE_ACTION_COOLING;
        } else if (this->mode_ == Mode::HEATING) {
          new_action = climate::CLIMATE_ACTION_HEATING;
        } else if (this->mode_ == Mode::AUTO) {
          // AUTO requires additional inference
          if (boiler && !chiller)
            new_action = climate::CLIMATE_ACTION_HEATING;
          else if (chiller && !boiler)
            new_action = climate::CLIMATE_ACTION_COOLING;
          else
            new_action = climate::CLIMATE_ACTION_IDLE;  // Both off or invalid
        } else {
          new_action = climate::CLIMATE_ACTION_IDLE;
        }
      } else {
        new_action = climate::CLIMATE_ACTION_IDLE;
      }

      if (this->action != new_action) {
        this->action = new_action;
        ESP_LOGD(TAG, "[%s] Updated action from valve status (reg 9 = 0x%04X): ev1=%d boiler=%d chiller=%d → %s",
                this->get_name().c_str(), reg9, ev1, boiler, chiller,
                climate::climate_action_to_string(new_action));
      } else {
        ESP_LOGD(TAG, "[%s] Valve status unchanged (reg 9 = 0x%04X): action=%s",
                this->get_name().c_str(), reg9,
                climate::climate_action_to_string(this->action));
      }

      // Always publish state, even if unchanged
      this->publish_state();
    });
}

// --- Periodic Polling Cycle (Valve Status + Water Temp) ---
void OlimpiaBridgeClimate::status_poll_cycle() {
  const uint32_t now = millis();

  // Skip polling until boot recovery is done
  if (!this->boot_recovery_done_)
    return;

  // Call update of climate action based on valve
  if (now - this->last_valve_status_poll_ > 30000UL) {
    this->last_valve_status_poll_ = now;
    this->update_climate_action_from_valve_status();
  }

  // Call update of water temperature sensor
  if (now - this->last_water_temp_poll_ > 30000UL) {
    this->last_water_temp_poll_ = now;
    this->read_water_temperature();
  }
}

// --- Conditional State Persistence ---
void OlimpiaBridgeClimate::maybe_save_state() {
  this->last_saved_state_ = {
    .on = this->on_,
    .mode = this->mode_,
    .fan_speed = this->fan_speed_,
    .target_temperature = this->target_temperature_,
    .last_action = this->action
  };

  if (this->saved_state_pref_.save(&this->last_saved_state_)) {
    ESP_LOGD(TAG, "[%s] Climate state saved to flash", this->get_name().c_str());
  } else {
    ESP_LOGW(TAG, "[%s] Failed to save climate state to flash", this->get_name().c_str());
  }
}

}  // namespace olimpia_bridge
}  // namespace esphome
