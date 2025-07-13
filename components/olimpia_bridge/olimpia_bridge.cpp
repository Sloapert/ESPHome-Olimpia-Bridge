#include <vector>
#include <utility>
#include "esphome.h"
#include "esphome/core/log.h"
#include "olimpia_bridge.h"
#include "olimpia_bridge_climate.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "orchestrator";

// --- Component Setup ---
void OlimpiaBridge::setup() {
  ESP_LOGI(TAG, "Setting up Olimpia Bridge");

  // Initialize RE/DE direction control pins
  if (this->re_pin_ != nullptr && this->de_pin_ != nullptr) {
    this->re_pin_->setup();
    this->de_pin_->setup();
    this->re_pin_->digital_write(false);  // RX mode
    this->de_pin_->digital_write(false);  // RX mode
  } else {
    ESP_LOGE(TAG, "No RE/DE pair configured.");
    this->mark_failed();
    return;
  }

  // Connect Modbus handler to UART and control pins
  if (this->handler_) {
    this->handler_->set_uart(this->uart_);
    this->handler_->set_re_pin(this->re_pin_);
    this->handler_->set_de_pin(this->de_pin_);
  }

  // Register handler as a component so loop() is called
  App.register_component(this->handler_);
  ESP_LOGI(TAG, "[Service] ModbusAsciiHandler initialized and registered");

  // Register custom Home Assistant services
  this->register_service(&OlimpiaBridge::read_register, "read_register", {"address", "register"});
  this->register_service(&OlimpiaBridge::write_register, "write_register", {"address", "register", "value"});
  this->register_service(&OlimpiaBridge::dump_configuration, "dump_config", {"address"});

  ESP_LOGI(TAG, "OlimpiaBridge setup complete");

  // Trigger initial sync for each attached climate entity
  for (auto *climate : this->climates_) {
    climate->control_cycle();  // Kick off 101/102/103/1 sync cycle
  }
}

// --- Periodic Update Cycle ---
void OlimpiaBridge::update() {
  ESP_LOGD(TAG, "[Service] Running periodic control cycle for all climates...");

  // Update and publish error ratio sensor value
  if (this->error_ratio_sensor_ && this->handler_) {
    float error_ratio = this->handler_->get_error_ratio();
    this->error_ratio_sensor_->publish_state(error_ratio);
  }
}

// --- Home Assistant Service: Read Register ---
void OlimpiaBridge::read_register(int address, int reg) {
  ESP_LOGI(TAG, "[Service] Reading register %d on address %d", reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "[Service] Modbus handler not initialized; read_register skipped");
    return;
  }

  this->handler_->read_register(
    static_cast<uint8_t>(address),
    static_cast<uint16_t>(reg),
    1,
    [address, reg](bool success, std::vector<uint16_t> response) {
      if (!success) {
        ESP_LOGW(TAG, "[Service] Read FAILED: addr %d reg %d", address, reg);
        return;
      }

      if (response.empty()) {
        ESP_LOGW(TAG, "[Service] Read OK but response empty: addr %d reg %d", address, reg);
        return;
      }

      uint16_t value = response[0];
      ESP_LOGI(TAG, "[Service] Read OK: addr %d reg %d → 0x%04X (%d)", address, reg, value, value);

      // Optional: handle value (e.g., update sensor, publish state)
    }
  );
}

// --- Home Assistant Service: Write Register ---
void OlimpiaBridge::write_register(int address, int reg, int value) {
  ESP_LOGI(TAG, "[Service] Writing value %d to register %d on address %d", value, reg, address);

  if (!this->handler_) {
    ESP_LOGW(TAG, "[Service] Modbus handler not initialized; write_register skipped");
    return;
  }

  this->handler_->write_register(
    static_cast<uint8_t>(address),
    static_cast<uint16_t>(reg),
    static_cast<uint16_t>(value),
    [address, reg, value](bool success, std::vector<uint16_t>) {
      if (!success) {
        ESP_LOGW(TAG, "[Service] Write FAILED: addr %d reg %d", address, reg);
        return;
      }

      ESP_LOGI(TAG, "[Service] Write OK: addr %d reg %d ← 0x%04X (%d)", address, reg, value, value);

      // Optional: confirm state, refresh read, etc.
    }
  );
}

// --- Home Assistant Service: Dump Configuration ---
void OlimpiaBridge::dump_configuration(int address) {
  uint8_t addr = static_cast<uint8_t>(address);  // Safely cast to uint8_t

  static uint16_t current_register = 0;
  static std::vector<std::pair<uint16_t, uint16_t>> results;  // (reg, value)

  if (current_register == 0) {
    results.clear();
    ESP_LOGI(TAG, "Started config dump for address %u in background, wait..", addr);
  }

  if (current_register > 255) {
    // All done — split log into chunks to avoid truncation
    constexpr size_t chunk_size = 30;
    size_t total = results.size();
    for (size_t i = 0; i < total; i += chunk_size) {
      std::string line;
      char header[32];
      snprintf(header, sizeof(header), "[0x%02X] DUMP:", addr);
      line += header;

      for (size_t j = i; j < i + chunk_size && j < total; ++j) {
        char buf[32];
        snprintf(buf, sizeof(buf), " R%03u=(%u)", results[j].first, results[j].second);
        line += buf;
      }

      ESP_LOGI(TAG, "%s", line.c_str());
    }

    current_register = 0;  // Reset for future use
    return;
  }

  // Read the current register
  uint16_t reg = current_register;
  this->handler_->read_register(addr, reg, 1, [this, addr, reg](bool ok, const std::vector<uint16_t> &data) {
    if (ok && !data.empty()) {
      results.emplace_back(reg, data[0]);
    } else {
      ESP_LOGW(TAG, "[0x%02X] Failed to read register %u", addr, reg);
      results.emplace_back(reg, 0xFFFF);  // Optionally flag failed reads
    }

    current_register++;

    // Continue after 30ms
    this->set_timeout("dump_config", 30, [this, addr]() {
      this->dump_configuration(static_cast<int>(addr));
    });
  });
}

// --- Climate Entity Registration ---
void OlimpiaBridge::add_climate(OlimpiaBridgeClimate *climate) {
  this->climates_.push_back(climate);
}

}  // namespace olimpia_bridge
}  // namespace esphome
