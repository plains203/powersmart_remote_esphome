#include "power_smart.h"
#include "esphome/core/log.h"

// ---------------------------------------------------------------------------
// Power Smart roller-shutter RF protocol for ESP32, driving ESPHome's
// official cc1101 component.
//
// This is a from-scratch reimplementation derived from, and numerically
// validated against, the open-source "power_smart" protocol decoder/encoder
// in the Flipper Zero firmware (lib/subghz/protocols/power_smart.c) and the
// real captured transmissions in that repository's test .sub files. The
// packet layout, checksum, and Manchester timing below were cross-checked by
// re-running the exact decode state machine against a real RF capture and
// confirming the encoder here reproduces the identical byte sequence and
// pulse train.
//
// Radio handling (SPI, registers, PA table, frequency synthesis, TX/IDLE
// state transitions) is delegated entirely to ESPHome's official cc1101
// component; this component only encodes the protocol and drives the
// bitstream through remote_transmitter (the ESP32 RMT peripheral) into the
// CC1101's GDO0 pin in async serial OOK mode - the same technique the
// Flipper Zero itself uses.
//
// Packet (64 bits, MSB first):
//   byte0 = 0xFD                          fixed sync
//   byte1 = K1(1) | CHANNEL(6) | ID_b15(1)
//   byte2 = ID_b14_8(7) | K2(1)
//   byte3 = ID_b7_0(8)
//   byte4 = 0xAA                          fixed sync
//   byte5:byte6 (16 bit) = ~(byte1:byte2)
//   byte7 = 0xFE - byte3
//
//   K1,K2 form a 2-bit command: 1=Down, 2=Up, 3=Stop
//   CHANNEL is a 6-bit one-hot value matching the remote's channel selector
//   ID is a 16-bit value unique to a given physical remote
//
// Line coding: standard Manchester, short=225us, long=450us. Each full
// 64-bit packet is sent 8x back-to-back (continuous Manchester stream, no
// reset between repeats), followed by a ~500ms silent gap. This whole unit
// is repeated `repeat_` times (default 10), matching what a real remote /
// the Flipper Zero replay produces.
// ---------------------------------------------------------------------------

namespace esphome {
namespace power_smart {

static const char *const TAG = "power_smart";

// Timing, validated against a real "Power Smart" capture
static const uint32_t TE_SHORT_US = 225;
static const uint32_t TE_LONG_US = 450;
static const uint32_t GAP_US = TE_LONG_US * 1111;  // ~500ms trailing silence between repeats
static const uint8_t INNER_REPEATS = 8;            // packet sent 8x back-to-back per burst

void PowerSmartComponent::setup() {
  if (this->radio_ == nullptr || this->transmitter_ == nullptr) {
    ESP_LOGE(TAG, "cc1101 and remote_transmitter must both be configured");
    this->mark_failed();
    return;
  }
}

void PowerSmartComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Power Smart transmitter:");
  ESP_LOGCONFIG(TAG, "  Repeat: %u", this->repeat_);
  ESP_LOGCONFIG(TAG, "  (radio settings: see cc1101 component's own log output)");
}

uint64_t PowerSmartComponent::build_packet_(uint16_t remote_id, uint8_t channel_mask, PowerSmartCommand command) {
  uint8_t cmd = static_cast<uint8_t>(command) & 0x03;
  uint8_t k1 = (cmd >> 1) & 1;
  uint8_t k2 = cmd & 1;
  uint8_t id_msb = (remote_id >> 15) & 1;
  uint8_t data_hi = (remote_id >> 8) & 0x7F;

  uint8_t byte0 = 0xFD;
  uint8_t byte1 = (k1 << 7) | ((channel_mask & 0x3F) << 1) | id_msb;
  uint8_t byte2 = (data_hi << 1) | k2;
  uint8_t byte3 = remote_id & 0xFF;
  uint8_t byte4 = 0xAA;

  uint16_t combined = (uint16_t(byte1) << 8) | byte2;
  uint16_t inv = (~combined) & 0xFFFF;
  uint8_t byte5 = (inv >> 8) & 0xFF;
  uint8_t byte6 = inv & 0xFF;
  uint8_t byte7 = (0xFE - byte3) & 0xFF;

  uint64_t data = 0;
  uint8_t bytes[8] = {byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7};
  for (uint8_t i = 0; i < 8; i++) {
    data = (data << 8) | bytes[i];
  }
  return data;
}

namespace {

enum SymbolResult { SHORT_LOW, LONG_LOW, LONG_HIGH, SHORT_HIGH };

struct EncState {
  uint8_t step{0};
  bool prev_bit{false};
};

// Re-implementation of Flipper's generic manchester_encoder_advance() state
// machine (lib/toolbox/manchester_encoder.c), which merges consecutive
// identical bits into a single "long" symbol instead of emitting two shorts.
bool encoder_advance(EncState &state, bool curr_bit, SymbolResult &result) {
  switch (state.step) {
    case 0:
      state.prev_bit = curr_bit;
      result = state.prev_bit ? SHORT_LOW : SHORT_HIGH;
      state.step = 1;
      return true;
    case 1: {
      uint8_t val = (uint8_t(state.prev_bit) << 1) | uint8_t(curr_bit);
      static const SymbolResult mapping[4] = {SHORT_LOW, LONG_LOW, LONG_HIGH, SHORT_HIGH};
      result = mapping[val];
      if (curr_bit == state.prev_bit) {
        state.step = 2;
        return false;
      } else {
        state.prev_bit = curr_bit;
        return true;
      }
    }
    case 2:
    default:
      result = curr_bit ? SHORT_LOW : SHORT_HIGH;
      state.prev_bit = curr_bit;
      state.step = 1;
      return true;
  }
}

void push_symbol(std::vector<int32_t> &out, SymbolResult r) {
  switch (r) {
    case SHORT_LOW:
      out.push_back(-int32_t(TE_SHORT_US));
      break;
    case LONG_LOW:
      out.push_back(-int32_t(TE_LONG_US));
      break;
    case LONG_HIGH:
      out.push_back(int32_t(TE_LONG_US));
      break;
    case SHORT_HIGH:
      out.push_back(int32_t(TE_SHORT_US));
      break;
  }
}

}  // namespace

void PowerSmartComponent::encode_manchester_(uint64_t data, uint8_t data_bits, uint8_t inner_repeats,
                                              std::vector<int32_t> &out) {
  EncState state;
  for (uint8_t rep = 0; rep < inner_repeats; rep++) {
    for (int8_t i = data_bits; i > 0; i--) {
      bool bit_val = !((data >> (i - 1)) & 1);
      SymbolResult result;
      if (!encoder_advance(state, bit_val, result)) {
        push_symbol(out, result);
        encoder_advance(state, bit_val, result);
      }
      push_symbol(out, result);
    }
  }
  SymbolResult final_result = state.prev_bit ? SHORT_HIGH : SHORT_LOW;
  if (final_result == SHORT_HIGH) {
    push_symbol(out, final_result);
  }
  out.push_back(-int32_t(GAP_US));
}

void PowerSmartComponent::send_command(uint16_t remote_id, uint8_t channel, PowerSmartCommand command) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Component failed at setup - not sending");
    return;
  }
  if (channel < 1 || channel > 6) {
    ESP_LOGE(TAG, "channel must be 1-6, got %u", channel);
    return;
  }
  uint8_t channel_mask = 1 << (channel - 1);

  uint64_t packet = build_packet_(remote_id, channel_mask, command);
  ESP_LOGD(TAG, "Sending remote_id=0x%04X channel=%u command=%u packet=0x%016llX", remote_id, channel,
           (unsigned) command, (unsigned long long) packet);

  std::vector<int32_t> pulses;
  encode_manchester_(packet, 64, INNER_REPEATS, pulses);

  // begin_tx() puts the CC1101 into async-serial TX mode with a proper
  // IDLE -> calibrate -> TX transition and MARCSTATE verification; the RMT
  // peripheral (remote_transmitter) then drives the bitstream into GDO0 and
  // the CC1101 keys the carrier on/off accordingly.
  this->radio_->begin_tx();

  auto call = this->transmitter_->transmit();
  call.get_data()->set_data(pulses);
  call.set_send_times(this->repeat_);
  call.set_send_wait(0);  // the trailing gap is already baked into `pulses`
  call.perform();

  this->radio_->set_idle();
}

}  // namespace power_smart
}  // namespace esphome
