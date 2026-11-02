import esphome.codegen as cg
import esphome.config_validation as cv

# Declare the base namespace so ESPHome registers it before pulling platforms
vl53l4cd_pololu_ns = cg.esphome_ns.namespace("vl53l4cd_pololu")

# Inform the build system that this component exists
CONFIG_SCHEMA = cv.Schema({})
