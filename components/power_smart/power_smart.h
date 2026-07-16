#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cc1101/cc1101.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"

#include <vector>

namespace esphome {
namespace power_smart {

// Command values match the "K1/K2" button bits decoded from real Power Smart
// remotes (verified against captured .sub files): 1=Down, 2=Up, 3=Stop.
enum PowerSmartCommand : uint8_t {
  POWER_SMART_COMMAND_UNKNOWN = 0,
  POWER_SMART_COMMAND_DOWN = 1,
  POWER_SMART_COMMAND_UP = 2,
  POWER_SMART_COMMAND_STOP = 3,
};

class PowerSmartComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_cc1101(cc1101::CC1101Component *radio) { this->radio_ = radio; }
  void set_remote_transmitter(remote_transmitter::RemoteTransmitterComponent *transmitter) {
    this->transmitter_ = transmitter;
  }
  void set_repeat(uint8_t repeat) { this->repeat_ = repeat; }

  // channel is 1-6 (matches the physical remote's 6-position channel selector)
  void send_command(uint16_t remote_id, uint8_t channel, PowerSmartCommand command);

 protected:
  static uint64_t build_packet_(uint16_t remote_id, uint8_t channel_mask, PowerSmartCommand command);
  static void encode_manchester_(uint64_t data, uint8_t data_bits, uint8_t inner_repeats,
                                  std::vector<int32_t> &out);

  cc1101::CC1101Component *radio_{nullptr};
  remote_transmitter::RemoteTransmitterComponent *transmitter_{nullptr};
  uint8_t repeat_{10};
};

template<typename... Ts> class PowerSmartSendCommandAction : public Action<Ts...> {
 public:
  PowerSmartSendCommandAction(PowerSmartComponent *parent) : parent_(parent) {}

  TEMPLATABLE_VALUE(uint16_t, remote_id)
  TEMPLATABLE_VALUE(uint8_t, channel)
  TEMPLATABLE_VALUE(PowerSmartCommand, command)

  void play(Ts... x) override {
    this->parent_->send_command(this->remote_id_.value(x...), this->channel_.value(x...),
                                 this->command_.value(x...));
  }

 protected:
  PowerSmartComponent *parent_;
};

}  // namespace power_smart
}  // namespace esphome
