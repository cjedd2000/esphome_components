import esphome.codegen as cg
from esphome.components import binary_sensor, sensor, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, DEVICE_CLASS_MOTION, ICON_EMPTY, ICON_MOTION_SENSOR

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor"]

LAST_ID = "last_id"
SENSOR_ID = "sensor_id"
MOTION_SENSORS = "motion_sensors"
MOTION_TIMEOUT = "motion_timeout"

htzsafe_owl_alarm_ns = cg.esphome_ns.namespace("htzsafe_owl_alarm")
HtzsafeOwlAlarm = htzsafe_owl_alarm_ns.class_(
    "HtzsafeOwlAlarm", cg.Component, uart.UARTDevice
)

# Binary Sensor Schema with added Sensor Id for setup
MotionSensorSchema = cv.Schema(
    {
        # cv.Optional("name"): cv.string,
        cv.Required(SENSOR_ID): cv.int_range(0, 65535),
        # Timeout in seconds, limit to 1 hour
        cv.Optional(MOTION_TIMEOUT): cv.int_range(1, 3600),
    }
).extend(
    binary_sensor.binary_sensor_schema(
        icon=ICON_MOTION_SENSOR, device_class=DEVICE_CLASS_MOTION
    )
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HtzsafeOwlAlarm),
            # Optional sensor to show last ID received, used to ID new sensors
            cv.Optional(LAST_ID): sensor.sensor_schema(
                icon=ICON_EMPTY, accuracy_decimals=0
            ),
            # List of Motions sensors and their ID
            cv.Optional(MOTION_SENSORS): cv.ensure_list(MotionSensorSchema),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if LAST_ID in config:
        sens = await sensor.new_sensor(config[LAST_ID])
        cg.add(var.set_last_id_sensor(sens))

    if MOTION_SENSORS in config:
        for item in config[MOTION_SENSORS]:
            sens = await binary_sensor.new_binary_sensor(item)

            if MOTION_TIMEOUT in item:
                cg.add(
                    var.add_motion_sensor_timeout(
                        sens, item[SENSOR_ID], item[MOTION_TIMEOUT] * 1000
                    )
                )
            else:
                cg.add(var.add_motion_sensor(sens, item[SENSOR_ID]))
