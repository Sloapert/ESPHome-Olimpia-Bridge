// --- OLIMPIA BRIDGE COMPONENT HEADER ---
#pragma once

#include <vector>
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/api/custom_api_device.h"
#include "modbus_ascii_handler.h"
#include "olimpia_bridge_climate.h"

namespace esphome {
namespace olimpia_bridge {

// --- OlimpiaBridge component (Modbus bridge + HA services) ---
class OlimpiaBridge : public PollingComponent, public api::CustomAPIDevice {
 public:
  OlimpiaBridge() : PollingComponent(60000) {}  // 60s polling

  // Climate Entity Management
  void add_climate(OlimpiaBridgeClimate *climate);
  std::vector<OlimpiaBridgeClimate *> climates_;

  // Component Lifecycle
  void setup() override;
  void update() override;

  // Hardware Configuration Setters
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_ = parent; }

  // Set GPIO pins from YAML
  void set_re_pin(GPIOPin *pin) {
    this->re_pin_ = pin;
    if (this->handler_ != nullptr)
      this->handler_->set_re_pin(pin);
  }
  void set_de_pin(GPIOPin *pin) {
    this->de_pin_ = pin;
    if (this->handler_ != nullptr)
      this->handler_->set_de_pin(pin);
  }

  void set_handler(ModbusAsciiHandler *handler) { this->handler_ = handler; }

  // Service methods callable from Home Assistant
  void read_register(int address, int reg);
  void write_register(int address, int reg, int value);

 protected:
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *re_pin_{nullptr};
  GPIOPin *de_pin_{nullptr};
  ModbusAsciiHandler *handler_{nullptr};
};

}  // namespace olimpia_bridge
}  // namespace esphome
