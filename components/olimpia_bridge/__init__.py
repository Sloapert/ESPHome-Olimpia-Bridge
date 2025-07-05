# --- ESPHOME PYTHON BINDINGS ---
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, sensor
from esphome import pins
from esphome.const import CONF_ID, CONF_UART_ID, CONF_NAME, CONF_ADDRESS

# --- Metadata ---
CODEOWNERS = ["@r0bb10"]
AUTO_LOAD = ["climate", "sensor", "uart"]
DEPENDENCIES = ["uart"]

# --- Custom configuration keys ---
CONF_HANDLER_ID = "handler"
CONF_CLIMATES = "climates"
CONF_WATER_TEMPERATURE_SENSOR = "water_temperature_sensor"
CONF_EMA_ALPHA = "ema_alpha"

# --- Define C++ class bindings ---
olimpia_bridge_ns = cg.esphome_ns.namespace("olimpia_bridge")
OlimpiaBridge = olimpia_bridge_ns.class_("OlimpiaBridge", cg.Component, uart.UARTDevice)
OlimpiaBridgeClimate = olimpia_bridge_ns.class_("OlimpiaBridgeClimate", climate.Climate, cg.Component)
ModbusAsciiHandler = olimpia_bridge_ns.class_("ModbusAsciiHandler")

# --- Per-climate configuration schema ---
olimpia_bridge_climate_schema = climate.climate_schema(OlimpiaBridgeClimate).extend({
    cv.Required(CONF_NAME): cv.string,
    cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=247),
    cv.Optional(CONF_EMA_ALPHA, default=0.2): cv.float_,
    cv.Optional(CONF_WATER_TEMPERATURE_SENSOR): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
})

# --- Top-level configuration schema ---
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OlimpiaBridge),
    cv.GenerateID(CONF_HANDLER_ID): cv.declare_id(ModbusAsciiHandler),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required("re_pin"): pins.gpio_output_pin_schema,
    cv.Required("de_pin"): pins.gpio_output_pin_schema,
    cv.Required(CONF_CLIMATES): cv.ensure_list(olimpia_bridge_climate_schema),
}).extend(cv.COMPONENT_SCHEMA)

# --- Code generation logic ---
async def to_code(config):
    # Instantiate OlimpiaBridge controller
    controller = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(controller, config)

    # Bind UART to OlimpiaBridge
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    cg.add(controller.set_uart_parent(uart_var))

    # Configure RE/DE GPIO pins
    re_pin = await cg.gpio_pin_expression(config["re_pin"])
    de_pin = await cg.gpio_pin_expression(config["de_pin"])
    cg.add(controller.set_re_pin(re_pin))
    cg.add(controller.set_de_pin(de_pin))

    # Create and assign ModbusAsciiHandler
    handler = cg.new_Pvariable(config[CONF_HANDLER_ID])
    cg.add(controller.set_handler(handler))

    # Process each climate entity in configuration
    for climate_conf in config[CONF_CLIMATES]:
        climate_var = cg.new_Pvariable(climate_conf[CONF_ID])
        await climate.register_climate(climate_var, climate_conf)
        await cg.register_component(climate_var, climate_conf)

        cg.add(climate_var.set_address(climate_conf[CONF_ADDRESS]))
        cg.add(climate_var.set_handler(handler))
        cg.add(controller.add_climate(climate_var))
        cg.add(climate_var.set_ema_alpha(climate_conf[CONF_EMA_ALPHA]))

        # Optional water temperature sensor
        if CONF_WATER_TEMPERATURE_SENSOR in climate_conf:
            sens = await sensor.new_sensor(climate_conf[CONF_WATER_TEMPERATURE_SENSOR])
            cg.add(climate_var.set_water_temp_sensor(sens))
