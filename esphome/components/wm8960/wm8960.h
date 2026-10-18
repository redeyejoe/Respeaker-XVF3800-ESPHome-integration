#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"

namespace esphome {
namespace wm8960 {

// Register numbers from the Cirrus/Wolfson WM8960 datasheet (Rev 4.4, register map table).
static const uint8_t WM8960_LINVOL = 0x00;      // R0  left input PGA volume
static const uint8_t WM8960_RINVOL = 0x01;      // R1  right input PGA volume
static const uint8_t WM8960_LOUT1 = 0x02;       // R2  left headphone volume
static const uint8_t WM8960_ROUT1 = 0x03;       // R3  right headphone volume
static const uint8_t WM8960_CLOCK1 = 0x04;      // R4  clocking 1 (ADCDIV, DACDIV, SYSCLKDIV, CLKSEL)
static const uint8_t WM8960_DACCTL1 = 0x05;     // R5  ADC & DAC control 1 (DACMU, ADCHPD)
static const uint8_t WM8960_IFACE1 = 0x07;      // R7  audio interface 1 (MS, WL, FORMAT)
static const uint8_t WM8960_CLOCK2 = 0x08;      // R8  clocking 2 (DCLKDIV, BCLKDIV)
static const uint8_t WM8960_IFACE2 = 0x09;      // R9  audio interface 2 (ALRCGPIO)
static const uint8_t WM8960_LDAC = 0x0A;        // R10 left DAC digital volume
static const uint8_t WM8960_RDAC = 0x0B;        // R11 right DAC digital volume
static const uint8_t WM8960_RESET = 0x0F;       // R15 software reset
static const uint8_t WM8960_LADC = 0x15;        // R21 left ADC digital volume
static const uint8_t WM8960_RADC = 0x16;        // R22 right ADC digital volume
static const uint8_t WM8960_ADDCTL2 = 0x18;     // R24 additional control 2 (LRCM)
static const uint8_t WM8960_POWER1 = 0x19;      // R25 power management 1
static const uint8_t WM8960_POWER2 = 0x1A;      // R26 power management 2
static const uint8_t WM8960_LINPATH = 0x20;     // R32 ADCL signal path
static const uint8_t WM8960_RINPATH = 0x21;     // R33 ADCR signal path
static const uint8_t WM8960_LOUTMIX = 0x22;     // R34 left output mixer
static const uint8_t WM8960_ROUTMIX = 0x25;     // R37 right output mixer
static const uint8_t WM8960_LOUT2 = 0x28;       // R40 left speaker volume
static const uint8_t WM8960_ROUT2 = 0x29;       // R41 right speaker volume
static const uint8_t WM8960_POWER3 = 0x2F;      // R47 power management 3
static const uint8_t WM8960_CLASSD1 = 0x31;     // R49 class D control 1
static const uint8_t WM8960_CLASSD3 = 0x33;     // R51 class D control 3

class WM8960 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;
  bool is_muted() override { return this->is_muted_; }
  float volume() override { return this->volume_; }

  void set_mic_boost(uint8_t boost) { this->mic_boost_ = boost; }
  void set_mic_gain_db(float db) { this->mic_gain_db_ = db; }
  void set_speaker_db(float db) { this->speaker_db_ = db; }

 protected:
  // The WM8960 takes 7-bit register addresses with 9-bit values, packed into two bytes.
  bool write_reg_(uint8_t reg, uint16_t value);
  bool write_dac_volume_();

  uint8_t mic_boost_{3};
  float mic_gain_db_{12.0f};
  float speaker_db_{0.0f};
  float volume_{0.5f};
};

}  // namespace wm8960
}  // namespace esphome
