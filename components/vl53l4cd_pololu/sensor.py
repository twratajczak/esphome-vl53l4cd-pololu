import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIMETER,
    UNIT_EMPTY,
)

# Namespace and Class bindings
vl53l4cd_pololu_ns = cg.esphome_ns.namespace("vl53l4cd_pololu")
VL53L4CDPololuComponent = vl53l4cd_pololu_ns.class_(
    "VL53L4CDPololuComponent", cg.Component, i2c.I2CDevice
)

DEPENDENCIES = ["i2c"]

# Custom configuration key names
CONF_TIMING_BUDGET = "timing_budget"
CONF_INTER_MEASUREMENT = "inter_measurement"
CONF_DISTANCE = "distance"
CONF_RANGE_STATUS = "range_status"
CONF_SIGNAL_RATE = "signal_rate"
CONF_AMBIENT_RATE = "ambient_rate"
CONF_FRAMES = "frames"
CONF_RECOVERY_COUNT = "recovery_count"

UNIT_KILO_COUNTS_PER_SECOND = "kcps"

# Configuration Schema with strict boundaries tailored to the VL53L4CD
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VL53L4CDPololuComponent),
            cv.Optional(CONF_TIMING_BUDGET, default="50ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=10), max=cv.TimePeriod(milliseconds=200)),
            ),
            cv.Optional(CONF_INTER_MEASUREMENT, default="0ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
            
            # Diagnostic & Telemetry Sensor Definitions
            cv.Required(CONF_DISTANCE): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLIMETER,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RANGE_STATUS): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_SIGNAL_RATE): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILO_COUNTS_PER_SECOND,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_AMBIENT_RATE): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILO_COUNTS_PER_SECOND,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FRAMES): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RECOVERY_COUNT): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x29))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Convert complex time intervals down to raw numerical types for C++ bindings
    cg.add(var.set_timing_budget(int(config[CONF_TIMING_BUDGET].total_milliseconds)))
    cg.add(var.set_inter_measurement(int(config[CONF_INTER_MEASUREMENT].total_milliseconds)))
    cg.add(var.set_update_interval(int(config[CONF_UPDATE_INTERVAL].total_milliseconds)))

    # Process and link each sub-sensor to its designated C++ class registry setter
    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    if CONF_RANGE_STATUS in config:
        sens = await sensor.new_sensor(config[CONF_RANGE_STATUS])
        cg.add(var.set_range_status_sensor(sens))

    if CONF_SIGNAL_RATE in config:
        sens = await sensor.new_sensor(config[CONF_SIGNAL_RATE])
        cg.add(var.set_signal_rate_sensor(sens))

    if CONF_AMBIENT_RATE in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT_RATE])
        cg.add(var.set_ambient_rate_sensor(sens))

    if CONF_FRAMES in config:
        sens = await sensor.new_sensor(config[CONF_FRAMES])
        cg.add(var.set_frames_sensor(sens))

    if CONF_RECOVERY_COUNT in config:
        sens = await sensor.new_sensor(config[CONF_RECOVERY_COUNT])
        cg.add(var.set_recovery_count_sensor(sens))

