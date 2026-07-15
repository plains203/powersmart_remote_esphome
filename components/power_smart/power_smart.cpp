#include "power_smart.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// ---------------------------------------------------------------------------
// Power Smart roller-shutter RF protocol, reimplemented for ESP32 + CC1101.
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
// Line coding: standard Manchester (IEEE-style), symbol timing:
//   short = 225us, long = 450us
// Each full 64-bit packet is sent 8x back-to-back (continuous Manchester
// stream, no reset between repeats), followed by a ~500ms silent gap. This
// whole unit is repeated `repeat_` times (default 10), matching what a real
// remote / the Flipper Zero replay produces.
// ---------------------------------------------------------------------------

namespace esphome {
namespace power_smart {

static const char *const TAG = "power_smart";

// CC1101 register addresses (standard TI CC1101 datasheet addresses)
static const uint8_t CC1101_IOCFG0 = 0x02;
static const uint8_t CC1101_FIFOTHR = 0x03;
static const uint8_t CC1101_PKTCTRL0 = 0x08;
static const uint8_t CC1101_FSCTRL1 = 0x0B;
static const uint8_t CC1101_FREQ2 = 0x0D;
static const uint8_t CC1101_FREQ1 = 0x0E;
static const uint8_t CC1101_FREQ0 = 0x0F;
static const uint8_t CC1101_MDMCFG4 = 0x10;
static const uint8_t CC1101_MDMCFG3 = 0x11;
static const uint8_t CC1101_MDMCFG2 = 0x12;
static const uint8_t CC1101_MDMCFG1 = 0x13;
static const uint8_t CC1101_MDMCFG0 = 0x14;
static const uint8_t CC1101_MCSM0 = 0x18;
static const uint8_t CC1101_FOCCFG = 0x19;
static const uint8_t CC1101_AGCCTRL2 = 0x1B;
static const uint8_t CC1101_AGCCTRL1 = 0x1C;
static const uint8_t CC1101_AGCCTRL0 = 0x1D;
static const uint8_t CC1101_WORCTRL = 0x20;
static const uint8_t CC1101_FREND1 = 0x21;
static const uint8_t CC1101_FREND0 = 0x22;
static const uint8_t CC1101_PATABLE = 0x3E;

// Strobe commands
static const uint8_t CC1101_SRES = 0x30;
static const uint8_t CC1101_SCAL = 0x33;
static const uint8_t CC1101_STX = 0x35;
static const uint8_t CC1101_SIDLE = 0x36;

static const uint8_t CC1101_READ_SINGLE = 0x80;
static const uint8_t CC1101_WRITE_BURST = 0x40;

// Timing, validated against a real "Power Smart" capture
static const uint32_t TE_SHORT_US = 225;
static const uint32_t TE_LONG_US = 450;
static const uint32_t GAP_US = TE_LONG_US * 1111;  // ~500ms trailing silence between repeats
static const uint8_t INNER_REPEATS = 8;            // packet sent 8x back-to-back per burst

void PowerSmartComponent::write_reg_(uint8_t addr, uint8_t value) {
  this->enable();
  this->transfer_byte(addr & 0x3F);
  this->transfer_byte(value);
  this->disable();
}

uint8_t PowerSmartComponent::read_reg_(uint8_t addr) {
  this->enable();
  this->transfer_byte((addr & 0x3F) | CC1101_READ_SINGLE);
  uint8_t value = this->transfer_byte(0x00);
  this->disable();
  return value;
}

void PowerSmartComponent::strobe_(uint8_t cmd) {
  this->enable();
  this->transfer_byte(cmd);
  this->disable();
}

void PowerSmartComponent::write_patable_(uint8_t value) {
  // CC1101 uses PATABLE[0] for the OOK/ASK "off" (low) level and PATABLE[1]
  // for the "on" (high) level, since FREND0's PA_POWER field (0x11) selects
  // index 1. Both must be burst-written in the same CS assertion - writing
  // only one byte leaves the other at an undefined reset value, which either
  // leaves TX power uncontrolled or degrades OOK modulation depth.
  uint8_t patable[8] = {0x00, value, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  this->enable();
  this->transfer_byte(CC1101_PATABLE | CC1101_WRITE_BURST);
  for (uint8_t i = 0; i < 8; i++) {
    this->transfer_byte(patable[i]);
  }
  this->disable();
}

uint8_t PowerSmartComponent::patable_for_dbm_(int8_t dbm) {
  // ASK/OOK PATABLE values (TI CC1101 reference settings)
  if (dbm >= 12)
    return 0xC0;
  if (dbm >= 10)
    return 0xC5;
  if (dbm >= 7)
    return 0xCD;
  if (dbm >= 5)
    return 0x86;
  if (dbm >= 0)
    return 0x50;
  if (dbm >= -6)
    return 0x37;
  if (dbm >= -10)
    return 0x26;
  if (dbm >= -15)
    return 0x1D;
  if (dbm >= -20)
    return 0x17;
  return 0x03;
}

void PowerSmartComponent::set_frequency_registers_() {
  uint64_t freq_word = (uint64_t) this->frequency_hz_ * 65536ULL / this->crystal_hz_;
  this->write_reg_(CC1101_FREQ2, (freq_word >> 16) & 0xFF);
  this->write_reg_(CC1101_FREQ1, (freq_word >> 8) & 0xFF);
  this->write_reg_(CC1101_FREQ0, freq_word & 0xFF);
}

void PowerSmartComponent::configure_radio_() {
  this->strobe_(CC1101_SRES);
  delay(2);

  // Matches Flipper Zero's "OOK 650kHz Async" preset: async serial GDO0,
  // ASK/OOK modulation, no preamble/sync/whitening - the MCU drives the
  // bitstream directly and CC1101 just keys the carrier on/off.
  this->write_reg_(CC1101_IOCFG0, 0x0D);
  this->write_reg_(CC1101_FIFOTHR, 0x07);
  this->write_reg_(CC1101_PKTCTRL0, 0x32);
  this->write_reg_(CC1101_FSCTRL1, 0x06);
  this->write_reg_(CC1101_MDMCFG0, 0x00);
  this->write_reg_(CC1101_MDMCFG1, 0x00);
  this->write_reg_(CC1101_MDMCFG2, 0x30);
  this->write_reg_(CC1101_MDMCFG3, 0x32);
  this->write_reg_(CC1101_MDMCFG4, 0x17);
  this->write_reg_(CC1101_MCSM0, 0x18);
  this->write_reg_(CC1101_FOCCFG, 0x18);
  this->write_reg_(CC1101_AGCCTRL0, 0x91);
  this->write_reg_(CC1101_AGCCTRL1, 0x00);
  this->write_reg_(CC1101_AGCCTRL2, 0x07);
  this->write_reg_(CC1101_WORCTRL, 0xFB);
  this->write_reg_(CC1101_FREND0, 0x11);
  this->write_reg_(CC1101_FREND1, 0xB6);

  this->set_frequency_registers_();
  this->write_patable_(this->patable_for_dbm_(this->tx_power_dbm_));

  this->strobe_(CC1101_SCAL);
  delay(2);
  this->strobe_(CC1101_SIDLE);
}

void PowerSmartComponent::setup() {
  this->spi_setup();

  if (this->transmitter_ == nullptr) {
    ESP_LOGE(TAG, "No remote_transmitter configured - set remote_transmitter_id in YAML");
    this->mark_failed();
    return;
  }

  // Give the crystal oscillator time to stabilize before the first SPI
  // transaction. The datasheet's recommended approach is to poll MISO until
  // it goes low; ESPHome's SPIDevice abstraction doesn't expose a direct
  // MISO read, so a conservative fixed delay is used instead.
  delay(30);

  this->configure_radio_();

  // Sanity check: read back a register we just wrote. Catches wiring
  // mistakes (swapped MISO/MOSI, floating CS, no power to the module) at
  // boot instead of failing silently on every transmit attempt.
  uint8_t readback = this->read_reg_(CC1101_FREND1);
  if (readback != 0xB6) {
    ESP_LOGE(TAG, "CC1101 register read-back mismatch (got 0x%02X, expected 0xB6) - check wiring/power", readback);
    this->mark_failed();
  }
}

void PowerSmartComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Power Smart CC1101 transmitter:");
  ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", this->frequency_hz_ / 1000000.0f);
  ESP_LOGCONFIG(TAG, "  Crystal: %.3f MHz", this->crystal_hz_ / 1000000.0f);
  ESP_LOGCONFIG(TAG, "  TX power: %d dBm", this->tx_power_dbm_);
  ESP_LOGCONFIG(TAG, "  Repeat: %u", this->repeat_);
  LOG_PIN("  CS Pin: ", this->cs_);
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

  // Enter TX: GDO0 becomes the async serial data input, CC1101 keys the
  // carrier on/off according to whatever level the ESP32 drives on it.
  this->strobe_(CC1101_STX);
  delay(2);  // allow PLL/PA turn-on to settle before the first edge

  auto call = this->transmitter_->transmit();
  call.get_data()->set_data(pulses);
  call.set_send_times(this->repeat_);
  call.set_send_wait(0);  // the trailing gap is already baked into `pulses`
  call.perform();

  this->strobe_(CC1101_SIDLE);
}

}  // namespace power_smart
}  // namespace esphome
