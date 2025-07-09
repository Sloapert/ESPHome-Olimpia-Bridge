#include "olimpia_bridge_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "climate";

// --- Helper functions for logging ---
static const char *mode_to_string(Mode mode) {
  switch (mode) {
    case Mode::AUTO: return "AUTO";
    case Mode::COOLING: return "COOL";
    case Mode::HEATING: return "HEAT";
    case Mode::UNKNOWN:
    default: return "UNKNOWN";
  }
}

static const char *fan_speed_to_string(FanSpeed fan) {
  switch (fan) {
    case FanSpeed::AUTO: return "AUTO";
    case FanSpeed::MIN: return "LOW";
    case FanSpeed::NIGHT: return "QUIET";
    case FanSpeed::MAX: return "HIGH";
    case FanSpeed::UNKNOWN:
    default: return "UNKNOWN";
  }
}

static std::string presets_to_uppercase(const std::string &str) {
  std::string out = str;
  for (auto &c : out) c = toupper(c);
  return out;
}

// --- Compose Register 101: Bit-mapped control flags ---
uint16_t OlimpiaBridgeClimate::build_command_register(bool on, Mode mode, FanSpeed fan_speed) {
  uint16_t reg = 0;

  // Bits 0–2: PRG fan speed
  // 000 = Auto, 001 = Min, 010 = Night, 011 = Max
  reg |= static_cast<uint8_t>(fan_speed) & 0x07;

  // Bit 7: STBY (working condition)
  // 0 = Activated, 1 = Standby (OFF)
  if (!on)
    reg |= (1 << 7);

  // Bit 12: CP presence contact
  reg &= ~(1 << 12);  // Ensure CP is always 0

  // Bits 13–14: EI mode of functioning
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
  this->system_boot_time_ms_ = millis();

  // Load fallback ambient temperature from flash
  this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash() ^ 0x1030U);
  this->saved_state_pref_ = global_preferences->make_preference<SavedState>(this->get_object_id_hash() ^ 0x2040U);

  float fallback = NAN;
  bool has_fallback = this->pref_.load(&fallback);
  if (has_fallback) {
    this->external_ambient_temperature_ = fallback;
    this->smoothed_ambient_ = fallback;
    this->using_fallback_external_temp_ = true;
    ESP_LOGD(TAG, "[%s] Climate setup → fallback_used=true temp=%.2f°C", this->get_name().c_str(), fallback);
  } else {
    ESP_LOGD(TAG, "[%s] Climate setup → fallback_used=false (no flash value)", this->get_name().c_str());
  }

  // Load or initialize climate state from flash
  SavedState recovered;
  bool found = this->saved_state_pref_.load(&recovered);
  if (found) {
    this->on_ = recovered.on;
    this->mode_ = recovered.mode;
    this->fan_speed_ = recovered.fan_speed;
    this->target_temperature_ = recovered.target_temperature;
    this->last_saved_state_ = recovered;
    this->custom_preset_ = recovered.custom_preset;

    ESP_LOGI(TAG, "[%s] Recovered state from flash: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C",
             this->get_name().c_str(),
             recovered.on ? "ON" : "OFF",
             mode_to_string(static_cast<Mode>(recovered.mode)),
             fan_speed_to_string(static_cast<FanSpeed>(recovered.fan_speed)),
             presets_to_uppercase(recovered.custom_preset).c_str(),
             recovered.target_temperature);
  } else {
    // First boot or flash reset: use defaults
    this->on_ = false;
    this->mode_ = Mode::AUTO;  // Represent OFF via on_=false + AUTO
    this->fan_speed_ = FanSpeed::AUTO;
    this->target_temperature_ = 22.0f;
    this->custom_preset_ = "Auto";

    ESP_LOGW(TAG, "[%s] No saved user state found, applying default: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C",
             this->get_name().c_str(),
             this->on_ ? "ON" : "OFF",
             mode_to_string(this->mode_),
             fan_speed_to_string(this->fan_speed_),
             presets_to_uppercase(this->custom_preset_).c_str(),
             this->target_temperature_);

    SavedState default_state{
      .on = this->on_,
      .mode = this->mode_,
      .fan_speed = this->fan_speed_,
      .target_temperature = this->target_temperature_,
    };
    strncpy(default_state.custom_preset, this->custom_preset_.c_str(), sizeof(default_state.custom_preset) - 1);
    default_state.custom_preset[sizeof(default_state.custom_preset) - 1] = '\0';
    this->last_saved_state_ = default_state;
    this->saved_state_pref_.save(&default_state);
  }

  // Sync fan mode to Home Assistant UI
  switch (this->fan_speed_) {
    case FanSpeed::AUTO:  this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
    case FanSpeed::MIN:   this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case FanSpeed::NIGHT: this->fan_mode = climate::CLIMATE_FAN_QUIET; break;
    case FanSpeed::MAX:   this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default:              this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
  }

  this->target_temperature = this->target_temperature_;
  this->publish_state();

  // Read reg103 (ambient temp) or push fallback to device
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

  // Read water temp from device
  this->read_water_temperature();
  this->last_water_temp_poll_ = millis();

  // Start boot recovery or sync state
  this->restore_or_refresh_state();

  // Randomize per-device periodic poll intervals
  this->next_control_cycle_ms_ = millis() + random(0, 60000);  // 0–60s
  this->next_status_poll_ms_ = millis() + random(0, 30000);    // 0–30s
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
  traits.set_visual_target_temperature_step(this->target_temperature_step_);

  // Ensure min and max temperature traits are optional
  if (!std::isnan(this->min_temperature_)) {
    traits.set_visual_min_temperature(this->min_temperature_);
  }
  if (!std::isnan(this->max_temperature_)) {
    traits.set_visual_max_temperature(this->max_temperature_);
  }

  // Update traits to conditionally expose presets
  if (this->presets_enabled_) {
    traits.add_supported_custom_preset("Auto");
    traits.add_supported_custom_preset("Manual");
  }

  return traits;
}

// --- Home Assistant control callback ---
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
    switch (*call.get_mode()) {
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
    auto fan = *call.get_fan_mode();
    if (fan == climate::CLIMATE_FAN_AUTO) this->fan_speed_ = FanSpeed::AUTO;
    else if (fan == climate::CLIMATE_FAN_LOW) this->fan_speed_ = FanSpeed::MIN;
    else if (fan == climate::CLIMATE_FAN_QUIET) this->fan_speed_ = FanSpeed::NIGHT;
    else if (fan == climate::CLIMATE_FAN_HIGH) this->fan_speed_ = FanSpeed::MAX;
    state_changed = true;
  }

  // Handle custom preset change
  if (call.get_custom_preset().has_value()) {
    std::string preset = *call.get_custom_preset();
    if (preset == "Auto" || preset == "Manual") {
      this->custom_preset_ = preset;
      ESP_LOGI(TAG, "[%s] Virtual preset set to %s", this->get_name().c_str(), presets_to_uppercase(preset).c_str());
      state_changed = true;
    } else {
      ESP_LOGW(TAG, "[%s] Unsupported virtual preset: %s", this->get_name().c_str(), presets_to_uppercase(preset).c_str());
    }
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
      .last_action = this->action,
    };

    // Safely copy custom_preset_ into the char[16] array
    strncpy(current.custom_preset, this->custom_preset_.c_str(), sizeof(current.custom_preset) - 1);
    current.custom_preset[sizeof(current.custom_preset) - 1] = '\0'; // Ensure null termination

    // Only save to flash if something actually changed
    if (memcmp(&this->last_saved_state_, &current, sizeof(SavedState)) != 0) {
      this->last_saved_state_ = current;
      if (this->saved_state_pref_.save(&current)) {
        ESP_LOGD(TAG, "[%s] Updated user state saved to flash: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C",
                 this->get_name().c_str(),
                 this->on_ ? "ON" : "OFF",
                 mode_to_string(this->mode_),
                 fan_speed_to_string(this->fan_speed_),
                 presets_to_uppercase(this->custom_preset_).c_str(),
                 this->target_temperature_);
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

// --- Read Register 1 (water temperature) ---
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

  // Skip control cycle if boot recovery is still in progress
  if (!this->boot_recovery_done_) {
    ESP_LOGD(TAG, "[%s] Skipping control cycle: boot recovery not complete", this->get_name().c_str());
    return;
  }

  // Update every 60s or on first boot
  if (!this->boot_cycle_done_ || (now - this->last_update_time_ >= 60000)) {
    ESP_LOGD(TAG, "[%s] Starting control cycle", this->get_name().c_str());
    // Push control values to registers 101/102/103
    this->write_control_registers_cycle();
    // Refresh current state from registers 101/102
    this->restore_or_refresh_state();
    // Update timestamps
    this->last_update_time_ = now;
    this->boot_cycle_done_ = true;
  }
}

// --- FSM Write Sequence: Reg 101 → 102 → 103 ---
void OlimpiaBridgeClimate::write_control_registers_cycle(std::function<void()> callback) {
  if (this->handler_ == nullptr) return;

  uint16_t reg101 = this->get_status_register();
  uint16_t reg102 = static_cast<uint16_t>(this->target_temperature_ * 10);
  uint16_t reg103 = std::isnan(this->external_ambient_temperature_) ? 0 : static_cast<uint16_t>(this->external_ambient_temperature_ * 10);

  ESP_LOGI(TAG, "[%s] Writing control → Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C | Ambient: %.1f°C",
           this->get_name().c_str(),
           this->on_ ? "ON" : "OFF",
           mode_to_string(this->mode_),
           fan_speed_to_string(this->fan_speed_),
           presets_to_uppercase(this->custom_preset_).c_str(),
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

        // Allow device time to process before action check
        if (callback) {
          this->set_timeout("valve_status_check", 500, [callback]() {
            callback();
          });
        }
      });
    });
  });
}

// --- Parse and Apply Register 101 State ---
void OlimpiaBridgeClimate::update_state_from_parsed(const ParsedState &parsed) {
  // Copy state from parsed register 101
  this->on_ = parsed.on;
  this->mode_ = parsed.mode;
  this->fan_speed_ = parsed.fan_speed;

  // Map internal state to HA climate mode
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
      default:
        this->mode = climate::CLIMATE_MODE_AUTO;
        break;
    }
  }

  // Map fan speed to HA fan mode
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

  // Push temperatures to ESPHome climate state
  this->current_temperature = this->external_ambient_temperature_;
  this->target_temperature = this->target_temperature_;  // use internal as reference for now

  // Apply custom preset
  if (this->custom_preset_ == "Auto" || this->custom_preset_ == "Manual") {
    this->custom_preset = this->custom_preset_;
  } else {
    this->custom_preset.reset();
  }

  ESP_LOGD(TAG, "[%s] Updated state from reg101: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C | Ambient: %.1f°C",
           this->get_name().c_str(),
           this->on_ ? "ON" : "OFF",
           mode_to_string(this->mode_),
           fan_speed_to_string(this->fan_speed_),
           presets_to_uppercase(this->custom_preset_).c_str(),
           this->target_temperature,
           this->current_temperature);
  // Update action if requested
  this->update_climate_action_from_valve_status();
  // Push state to HA
  this->publish_state();
}

// --- Compose Register 101 from State ---
uint16_t OlimpiaBridgeClimate::get_status_register() {
  return this->build_command_register(this->on_, this->mode_, this->fan_speed_);
}

// --- External Ambient Temperature from HA ---
void OlimpiaBridgeClimate::set_external_ambient_temperature(float temp) {
  if (std::isnan(temp)) return;

  const uint32_t now = millis();
  bool first_time = !this->has_received_external_temp_;
  bool refresh_flash = (now - this->last_external_temp_flash_write_ > 3600000UL);
  bool temp_changed = std::abs(temp - this->external_ambient_temperature_) > 0.05f;

  constexpr uint32_t EMA_INACTIVITY_RESET_MS = 15 * 60 * 1000UL;  // 15 minutes

  // Note: first_ha_ambient_received_ must stay false after fallback,
  // so first HA value is bypassed (and logs accordingly), but then enables EMA.

  // Handle fallback and first-HA reception
  if (!this->has_received_external_temp_ && this->using_fallback_external_temp_) {
    ESP_LOGI(TAG, "[%s] Restoring last known ambient from FLASH: %.1f°C", this->get_name().c_str(), temp);
    this->first_ha_ambient_received_ = false;  // Ensure bypass still happens on next HA update
    this->smoothed_ambient_ = NAN;  // Defensive reset for EMA after reboots
  } else if (!this->first_ha_ambient_received_) {
    ESP_LOGI(TAG, "[%s] Fresh ambient received from HA: %.1f°C, enabling EMA!", this->get_name().c_str(), temp);
    this->first_ha_ambient_received_ = true;
    this->smoothed_ambient_ = NAN;  // Reset EMA
  } else {
    // Smart EMA Reset if inactive too long
    if (now - this->last_external_temp_update_ > EMA_INACTIVITY_RESET_MS) {
      ESP_LOGI(TAG, "[%s] EMA reset due to inactivity. Accepting new ambient: %.1f°C", this->get_name().c_str(), temp);
      this->external_ambient_temperature_ = temp;
      this->current_temperature = temp;
      this->smoothed_ambient_ = temp;
      this->has_received_external_temp_ = true;
      this->external_temp_received_from_ha_ = true;
      this->last_external_temp_update_ = now;
      this->publish_state();
      return;
    }

    // Exponential Moving Average smoothing logic with trend-based early confirmation logic
    float prev = this->smoothed_ambient_;
    float ema = std::isnan(prev) ? temp : this->ambient_ema_alpha_ * temp + (1.0f - this->ambient_ema_alpha_) * prev;

    float rounded = std::round(ema * 10.0f) / 10.0f;
    float trend = ema - prev;

    this->smoothed_ambient_ = ema;

    if (rounded != this->external_ambient_temperature_) {
      bool should_confirm = false;
      if (rounded > this->external_ambient_temperature_) {
        should_confirm = trend > 0 && ema >= (rounded - 0.02f);
      } else if (rounded < this->external_ambient_temperature_) {
        should_confirm = trend < 0 && ema <= (rounded + 0.02f);
      }

      const char *trend_str = (trend > 0) ? "↑" : (trend < 0) ? "↓" : "→";

      if (!should_confirm) {
        ESP_LOGI(TAG, "[%s] EMA rejected %.1f°C from HA → EMA %.2f°C (trend %s, held %.1f°C)",
                 this->get_name().c_str(), temp, ema, trend_str, this->external_ambient_temperature_);
        return;
      }

      temp = rounded;
      ESP_LOGI(TAG, "[%s] EMA accepted %.1f°C from HA → EMA %.2f°C (trend %s)",
               this->get_name().c_str(), temp, ema, trend_str);
    } else {
      ESP_LOGI(TAG, "[%s] EMA stable: %.1f°C (EMA %.2f°C)", this->get_name().c_str(), temp, ema);
    }
  }
  // Update memory state
  this->external_ambient_temperature_ = temp;
  this->current_temperature = temp;
  this->has_received_external_temp_ = true;
  this->external_temp_received_from_ha_ = true;
  this->last_external_temp_update_ = now;

  if ((first_time || this->using_fallback_external_temp_ || refresh_flash) && temp_changed) {
    this->pref_.save(&temp);
    this->last_external_temp_flash_write_ = now;
    this->using_fallback_external_temp_ = false;
  }

  this->publish_state();
}

// --- Restore Saved State from Flash ---
void OlimpiaBridgeClimate::apply_last_known_state() {
  SavedState recovered{};
  if (this->saved_state_pref_.load(&recovered)) {
    this->last_saved_state_ = recovered;
    ESP_LOGI(TAG, "[%s] Recovered state from flash: Power: %s | Mode: %s | Fan: %s | Preset: %s | Target: %.1f°C",
         this->get_name().c_str(),
         recovered.on ? "ON" : "OFF",
         mode_to_string(static_cast<Mode>(recovered.mode)),
         fan_speed_to_string(static_cast<FanSpeed>(recovered.fan_speed)),
         presets_to_uppercase(recovered.custom_preset).c_str(),
         recovered.target_temperature);
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
  this->custom_preset_ = recovered.custom_preset;

  // Sync fan mode for HA UI
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

  // Sync operating mode for HA UI
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

  // Skip duplicate recovery
  if (is_first_boot && this->boot_recovery_in_progress_)
    return;

  if (is_first_boot)
    this->boot_recovery_in_progress_ = true;

  if (is_first_boot) {
    ESP_LOGI(TAG, "[%s] Boot state recovery: reading 101 + 102...", this->get_name().c_str());
  } else {
    ESP_LOGD(TAG, "[%s] Refreshing state from device...", this->get_name().c_str());
  }

  this->handler_->read_register(this->address_, 101, 1,
    [this, is_first_boot](bool ok101, const std::vector<uint16_t> &data101) {
      if (!ok101 || data101.empty()) {
        ESP_LOGW(TAG, "[%s] Failed to read register 101", this->get_name().c_str());
        if (is_first_boot)
          this->boot_recovery_in_progress_ = false;
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
            if (is_first_boot)
              this->boot_recovery_in_progress_ = false;
            return;
          }

          float target = data102[0] * 0.1f;
          ESP_LOGD(TAG, "[%s] Read 102 → target temperature: %.1f°C", this->get_name().c_str(), target);

          // --- Power-Loss Recovery ---
          if (is_first_boot && parsed.mode == Mode::AUTO && std::abs(target - 22.0f) < 0.2f) {
            ESP_LOGW(TAG, "[%s] Detected fallback state (AUTO + 22.0°C), restoring from flash", this->get_name().c_str());
            this->apply_last_known_state();
            this->write_control_registers_cycle();  // Push corrected state back to device

            // Mark recovery as done, even in fallback case
            this->boot_recovery_done_ = true;
            this->block_control_until_recovery_ = false;
            this->boot_recovery_in_progress_ = false;
            ESP_LOGI(TAG, "[%s] Boot recovery fallback applied. Enabling control.", this->get_name().c_str());
            return;
          }

          // Set recovered target temperature
          this->target_temperature_ = target;
          this->target_temperature = target;
          this->update_state_from_parsed(parsed);  // Update internal + publish to HA

          ESP_LOGD(TAG, "[%s] Updated state → ON=%d MODE=%d FAN=%d target=%.1f°C",
                   this->get_name().c_str(), this->on_, static_cast<int>(this->mode_),
                   static_cast<int>(this->fan_speed_), this->target_temperature_);

          if (is_first_boot) {
            this->boot_recovery_done_ = true;
            this->block_control_until_recovery_ = false;
            this->boot_recovery_in_progress_ = false;
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
      auto new_action = this->action;  // Start with current action as default
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

  // Skip polling until boot recovery is complete
  if (!this->boot_recovery_done_)
    return;

  // Poll valve status (register 9)
  if (now - this->last_valve_status_poll_ > 30000UL) {
    this->last_valve_status_poll_ = now;
    this->update_climate_action_from_valve_status();
  }

  // Poll water temperature (register 1)
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

// --- Main Loop: Schedule Updates ---
void OlimpiaBridgeClimate::loop() {
  const uint32_t now = millis();

  // Staggered 60s control cycle
  if (this->boot_recovery_done_ && now >= this->next_control_cycle_ms_) {
    this->next_control_cycle_ms_ = now + 60000 + random(0, 3000);  // 60s + jitter
    this->write_control_registers_cycle([this]() {
      this->update_climate_action_from_valve_status();  // Follow-up read
    });
  }

  // Staggered 30s polling cycle
  if (this->boot_recovery_done_ && now >= this->next_status_poll_ms_) {
    this->next_status_poll_ms_ = now + 30000 + random(0, 2000);  // 30s + jitter
    this->status_poll_cycle();
  }
}

}  // namespace olimpia_bridge
}  // namespace esphome
