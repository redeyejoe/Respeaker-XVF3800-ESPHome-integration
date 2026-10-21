#include "wm8960.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace wm8960 {

static const char *const TAG = "wm8960";

// The HAT has its own oscillator on MCLK: 12.288 MHz on Seeed's boards (seeed-2mic-voicecard overlay:
// clock-frequency = <12288000>), 24 MHz on some clones such as the Keyestudio KS0314. SYSCLK must be 12.288 MHz:
// taken straight from MCLK, or made by the PLL from 24 MHz.
// The codec is the I2S master: it makes BCLK and LRCLK from SYSCLK, and the ESP32 follows as secondary.
// 12.288 MHz / 256 / 3 = 16 kHz sample rate; 12.288 MHz / 12 = 1.024 MHz BCLK = 64 bit clocks per frame.
// Mixer and gain values follow Seeed's wm8960_asound.state for this board.

bool WM8960::write_reg_(uint8_t reg, uint16_t value) {
  uint8_t data[2] = {static_cast<uint8_t>((reg << 1) | ((value >> 8) & 0x01)), static_cast<uint8_t>(value & 0xFF)};
  if (this->write(data, 2) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Write to R%u failed", reg);
    return false;
  }
  return true;
}

void WM8960::setup() {
  // The codec has no ID register; a failed reset write means it is not on the bus.
  if (!this->write_reg_(WM8960_RESET, 0x000)) {
    this->mark_failed();
    return;
  }
  delay(10);

  // Bias first, then let VMID settle before turning on the outputs (fewer pops).
  // VMIDSEL=01 (2x50k), VREF, AINL, AINR, ADCL, ADCR, MICB
  this->write_reg_(WM8960_POWER1, 0x0FE);
  delay(100);

  // ADCDIV = DACDIV = /3 -> 16 kHz from a 12.288 MHz SYSCLK
  uint16_t clock1 = (0b011 << 6) | (0b011 << 3);
  if (this->mclk_hz_ == 24000000) {
    // PLL (datasheet PLL table, 12 MHz -> 12.288 MHz row): MCLK / 2 (PLLPRESCALE) = 12 MHz,
    // x 8.192 (N = 8, K = 0x3126E8, fractional) = 98.304 MHz, / 4 / 2 (SYSCLKDIV) = 12.288 MHz
    this->write_reg_(WM8960_PLL1, (1 << 5) | (1 << 4) | 8);
    this->write_reg_(WM8960_PLL2, 0x31);
    this->write_reg_(WM8960_PLL3, 0x26);
    this->write_reg_(WM8960_PLL4, 0xE8);
    this->write_reg_(WM8960_POWER2, 0x001);  // PLL_EN, before switching SYSCLK over to it
    delay(5);
    clock1 |= (0b10 << 1) | 1;               // SYSCLKDIV = /2, CLKSEL = PLL
  }
  this->write_reg_(WM8960_CLOCK1, clock1);
  // DCLKDIV default (/16, class D switching clock), BCLKDIV = /12 -> 1.024 MHz
  this->write_reg_(WM8960_CLOCK2, 0x1C0 | 0b1001);
  // Master mode, 32-bit words, I2S format
  this->write_reg_(WM8960_IFACE1, (1 << 6) | (0b11 << 2) | 0b10);
  // Only one LRCLK reaches the Pi header: the ADC uses DACLRC (ALRCGPIO), and LRCLK keeps running
  // while either converter is on (LRCM).
  this->write_reg_(WM8960_IFACE2, 1 << 6);
  this->write_reg_(WM8960_ADDCTL2, 1 << 2);

  // Power: DACL, DACR, LOUT1, ROUT1, SPKL, SPKR / LMIC, RMIC, LOMIX, ROMIX
  this->write_reg_(WM8960_POWER2, 0x1F8 | (this->mclk_hz_ == 24000000 ? 0x001 : 0x000));
  this->write_reg_(WM8960_POWER3, 0x03C);

  // Mic inputs: LINPUT1 / RINPUT1 to the inverting PGA input, boost stage on, PGA into the boost mixer
  uint16_t path = (1 << 8) | ((this->mic_boost_ & 0x03) << 4) | (1 << 3);
  this->write_reg_(WM8960_LINPATH, path);
  this->write_reg_(WM8960_RINPATH, path);
  // PGA volume: 0x17 = 0 dB, 0.75 dB per step, unmuted; IPVU latches both sides
  int pga = static_cast<int>(lroundf(this->mic_gain_db_ / 0.75f)) + 0x17;
  pga = clamp(pga, 0, 0x3F);
  this->write_reg_(WM8960_LINVOL, pga);
  this->write_reg_(WM8960_RINVOL, 0x100 | pga);
  // ADC digital volume 0 dB (0xC3), ADCVU latches
  this->write_reg_(WM8960_LADC, 0x0C3);
  this->write_reg_(WM8960_RADC, 0x1C3);

  // DAC into the output mixers (LD2LO / RD2RO)
  this->write_reg_(WM8960_LOUTMIX, 0x150);
  this->write_reg_(WM8960_ROUTMIX, 0x150);
  // Headphone 0 dB; speaker level from config (0x79 = 0 dB, 1 dB steps); OUT1VU / SPKVU latch
  this->write_reg_(WM8960_LOUT1, 0x079);
  this->write_reg_(WM8960_ROUT1, 0x179);
  int spk = clamp(static_cast<int>(lroundf(this->speaker_db_)) + 0x79, 0x30, 0x7F);
  this->write_reg_(WM8960_LOUT2, spk);
  this->write_reg_(WM8960_ROUT2, 0x100 | spk);
  // Class D: both speaker outputs on; DC/AC boost 4/5 as Seeed ships it for a 5 V speaker supply
  this->write_reg_(WM8960_CLASSD1, 0x0F7);
  this->write_reg_(WM8960_CLASSD3, 0x080 | (4 << 3) | 5);

  // Unmute and set the starting volume
  this->write_dac_volume_();
  this->write_reg_(WM8960_DACCTL1, this->is_muted_ ? (1 << 3) : 0);
}

void WM8960::dump_config() {
  ESP_LOGCONFIG(TAG, "WM8960:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG,
                "  MCLK: %.3f MHz%s\n  Mic boost: %u (0/13/20/29 dB)\n  Mic gain: %.2f dB\n  Speaker level: %.0f dB",
                this->mclk_hz_ / 1e6f, this->mclk_hz_ == 24000000 ? " (through the PLL)" : "", this->mic_boost_,
                this->mic_gain_db_, this->speaker_db_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  No answer at this address. Check SDA/SCL and that the HAT has 3.3 V and 5 V.");
  }
}

bool WM8960::set_mute_off() {
  this->is_muted_ = false;
  return this->write_reg_(WM8960_DACCTL1, 0);
}

bool WM8960::set_mute_on() {
  this->is_muted_ = true;
  return this->write_reg_(WM8960_DACCTL1, 1 << 3);
}

bool WM8960::set_volume(float volume) {
  this->volume_ = clamp(volume, 0.0f, 1.0f);
  return this->write_dac_volume_();
}

bool WM8960::write_dac_volume_() {
  // DAC digital volume: 0xFF = 0 dB, 0.5 dB per step, 0x00 = mute. Map 0..1 onto -50..0 dB.
  uint8_t reg = 0;
  if (this->volume_ > 0.0f) {
    float db = (this->volume_ - 1.0f) * 50.0f;
    reg = static_cast<uint8_t>(clamp(static_cast<int>(lroundf(0xFF + db * 2.0f)), 1, 0xFF));
  }
  return this->write_reg_(WM8960_LDAC, reg) && this->write_reg_(WM8960_RDAC, 0x100 | reg);
}

}  // namespace wm8960
}  // namespace esphome
