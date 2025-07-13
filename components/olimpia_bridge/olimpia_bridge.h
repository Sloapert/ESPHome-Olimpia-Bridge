#pragma once

#include <vector>
#include "esphome/core/gpio.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"
#include "modbus_ascii_handler.h"

namespace esphome {
namespace olimpia_bridge {

class OlimpiaBridgeClimate;  // Forward declaration

// --- OlimpiaBridge Component ---
class OlimpiaBridge : public PollingComponent, public api::CustomAPIDevice {
 public:
  OlimpiaBridge() : PollingComponent(60000) {}  // 60s polling interval

  // Component lifecycle
  void setup() override;
  void update() override;

  // Climate entity management
  void add_climate(OlimpiaBridgeClimate *climate);
  std::vector<OlimpiaBridgeClimate *> climates_;

  // Hardware configuration
  void set_uart_parent(uart::UARTComponent *parent) { this->uart_ = parent; }

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

  void set_error_ratio_sensor(sensor::Sensor *sensor) { this->error_ratio_sensor_ = sensor; }

  // Home Assistant service methods
  void read_register(int address, int reg);
  void write_register(int address, int reg, int value);
  void dump_configuration(int address);

 protected:
  // Hardware references
  uart::UARTComponent *uart_{nullptr};
  GPIOPin *re_pin_{nullptr};
  GPIOPin *de_pin_{nullptr};

  // Modbus handler
  ModbusAsciiHandler *handler_{nullptr};

  // Error ratio sensor
  sensor::Sensor *error_ratio_sensor_{nullptr};
};

}  // namespace olimpia_bridge
}  // namespace esphome
