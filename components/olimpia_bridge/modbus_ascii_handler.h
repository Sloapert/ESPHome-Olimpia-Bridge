// --- MODBUS ASCII HANDLER HEADER ---
#pragma once

#include <queue>         // for std::queue
#include <vector>        // for std::vector
#include <functional>    // for std::function
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace olimpia_bridge {

class OlimpiaBridge;  // Forward declaration

// --- FSM States ---
enum class ModbusState {
  IDLE,
  SEND_REQUEST,
  WAIT_RESPONSE,
  PROCESS_RESPONSE,
  ERROR
};

// --- Modbus ASCII Handler Class ---
class ModbusAsciiHandler : public esphome::Component {
 public:
  // --- FSM: Request structure ---
  struct ModbusRequest {
    uint8_t address;
    uint8_t function;
    uint8_t retries_left = 2;  // New field: retry count
    uint16_t start_register;
    uint16_t length_or_value;
    bool is_write;
    std::function<void(bool success, std::vector<uint16_t> response)> callback;  // Async response handler
  };

  // --- Hardware Configuration ---
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }  // Set UART instance
  void set_re_pin(GPIOPin *pin) { this->re_pin_ = pin; }            // Set receiver enable pin
  void set_de_pin(GPIOPin *pin) { this->de_pin_ = pin; }            // Set driver enable pin
  void set_direction(bool transmit);                                // Set TX/RX line direction (true = TX, false = RX)

  // --- Modbus Request API ---
  void add_request(ModbusRequest request);  // Add new request to FSM queue
  void write_byte(uint8_t byte);            // Low-level UART write
  void loop() override;                     // FSM loop (called by ESPHome)

  // FSM fields
  ModbusState fsm_state_{ModbusState::IDLE};
  uint32_t fsm_start_time_{0};
  static constexpr uint32_t fsm_timeout_ms_{500};  // Incoming byte buffer (FSM RX)

  std::queue<ModbusRequest> request_queue_;
  ModbusRequest current_request_;

  // --- Public Modbus API (async) ---
  void read_register(uint8_t address, uint16_t reg, uint16_t count,
                    std::function<void(bool success, std::vector<uint16_t> response)> callback);

  void write_register(uint8_t address, uint16_t reg, uint16_t value,
                      std::function<void(bool success, std::vector<uint16_t> response)> callback);

 protected:
  // --- Modbus ASCII Frame Encoding/Decoding ---
  std::string encode_ascii_frame(const std::vector<uint8_t> &data);  // Encode payload into ASCII frame
  bool decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data);  // Decode ASCII frame into raw bytes
  uint8_t compute_lrc(const std::vector<uint8_t> &data);  // Compute LRC checksum

  // --- Hardware Interfaces ---
  uart::UARTComponent *uart_{nullptr};   // Assigned UART component
  GPIOPin *re_pin_{nullptr};             // Receiver Enable pin
  GPIOPin *de_pin_{nullptr};             // Driver Enable pin

  // --- FSM Frame Construction ---
  std::vector<uint8_t> build_request_frame_ascii_(const std::vector<uint8_t> &data);

  // --- FSM Response Handling ---
  bool read_available_();

  std::vector<uint8_t> rx_buffer_;  // Incoming byte buffer (FSM RX)
};

}  // namespace olimpia_bridge
}  // namespace esphome
