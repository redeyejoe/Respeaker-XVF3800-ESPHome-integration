import esphome.codegen as cg
from esphome.components import i2c
from esphome.components.audio_dac import AudioDac
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@redeyejoe"]
DEPENDENCIES = ["i2c"]

CONF_MIC_BOOST = "mic_boost"
CONF_MIC_GAIN = "mic_gain"
CONF_SPEAKER_VOLUME = "speaker_volume"

wm8960_ns = cg.esphome_ns.namespace("wm8960")
WM8960 = wm8960_ns.class_("WM8960", AudioDac, cg.Component, i2c.I2CDevice)

# Mic boost stage in front of the PGA: 0, 13, 20 or 29 dB (Seeed ships 29 dB)
MIC_BOOST = {0: 0, 13: 1, 20: 2, 29: 3}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WM8960),
            cv.Optional(CONF_MIC_BOOST, default=29): cv.one_of(*MIC_BOOST, int=True),
            # Input PGA gain, -17.25 .. +30 dB in 0.75 dB steps (Seeed ships +12 dB)
            cv.Optional(CONF_MIC_GAIN, default=12.0): cv.float_range(min=-17.25, max=30.0),
            # Class-D speaker driver level, -73 .. +6 dB (the volume control works on the DAC in front of it)
            cv.Optional(CONF_SPEAKER_VOLUME, default=0.0): cv.float_range(min=-73.0, max=6.0),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x1A))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    cg.add(var.set_mic_boost(MIC_BOOST[config[CONF_MIC_BOOST]]))
    cg.add(var.set_mic_gain_db(config[CONF_MIC_GAIN]))
    cg.add(var.set_speaker_db(config[CONF_SPEAKER_VOLUME]))
