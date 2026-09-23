import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, output, text_sensor, switch, button
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
# Tell ESPHome to generate headers for switch and text_sensor
# since our C++ code relies on them.
AUTO_LOAD = ["button", "switch", "text_sensor"]

garage_door_ns = cg.esphome_ns.namespace("garage_door")
GarageDoorComponent = garage_door_ns.class_("GarageDoorComponent", cg.Component, uart.UARTDevice)
GarageDoorVentingSwitch = garage_door_ns.class_("GarageDoorVentingSwitch", switch.Switch)
GarageDoorLightSwitch = garage_door_ns.class_("GarageDoorLightSwitch", switch.Switch)
GarageDoorEmergencyStopButton = garage_door_ns.class_("GarageDoorEmergencyStopButton", button.Button)
GarageDoorImpulseButton = garage_door_ns.class_("GarageDoorImpulseButton", button.Button)

CONF_WRITE_ENABLE_PIN = "write_enable_pin"
CONF_STATE_SENSOR = "state_sensor"
CONF_VENTING_SWITCH = "venting_switch"
CONF_LIGHT_SWITCH = "light_switch"
CONF_EMERGENCY_STOP_BUTTON = "emergency_stop_button"
CONF_IMPULSE_BUTTON = "impulse_button"
# Eigene emulierte Bus-Adresse (Default 0x28, siehe garage_door.h). Nur ändern,
# falls es einen Adresskonflikt mit einem zweiten Gerät (z.B. echtem UAP1) am
# selben Bus gibt.
CONF_OWN_ADDRESS = "own_address"
# Von cover.py verwendet, um sich mit dieser Hub-Instanz zu verbinden
CONF_GARAGE_DOOR_ID = "garage_door_id"

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(GarageDoorComponent),
        cv.Required(CONF_WRITE_ENABLE_PIN): cv.use_id(output.BinaryOutput),
        cv.Optional(CONF_STATE_SENSOR): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_VENTING_SWITCH): switch.switch_schema(GarageDoorVentingSwitch),
        cv.Optional(CONF_LIGHT_SWITCH): switch.switch_schema(GarageDoorLightSwitch),
        cv.Optional(CONF_EMERGENCY_STOP_BUTTON): button.button_schema(GarageDoorEmergencyStopButton),
        cv.Optional(CONF_IMPULSE_BUTTON): button.button_schema(GarageDoorImpulseButton),
        cv.Optional(CONF_OWN_ADDRESS, default=0x28): cv.int_range(min=0, max=255),
    }).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    write_enable = await cg.get_variable(config[CONF_WRITE_ENABLE_PIN])
    cg.add(var.set_write_enable(write_enable))

    cg.add(var.set_own_address(config[CONF_OWN_ADDRESS]))

    if CONF_STATE_SENSOR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATE_SENSOR])
        cg.add(var.set_state_sensor(sens))

    if CONF_VENTING_SWITCH in config:
        sw = await switch.new_switch(config[CONF_VENTING_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(var.set_venting_switch(sw))

    if CONF_LIGHT_SWITCH in config:
        sw = await switch.new_switch(config[CONF_LIGHT_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(var.set_light_switch(sw))

    if CONF_EMERGENCY_STOP_BUTTON in config:
        btn = await button.new_button(config[CONF_EMERGENCY_STOP_BUTTON])
        cg.add(btn.set_parent(var))

    if CONF_IMPULSE_BUTTON in config:
        btn = await button.new_button(config[CONF_IMPULSE_BUTTON])
        cg.add(btn.set_parent(var))
