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
CONF_MIN_TEMPERATURE = "min_temperature"
CONF_MAX_TEMPERATURE = "max_temperature"
CONF_TARGET_TEMPERATURE_STEP = "target_temperature_step"
CONF_PRESETS_ENABLED = "presets"
CONF_DISABLE_MODE_AUTO = "disable_mode_auto"
CONF_USE_EMA = "use_ema"
CONF_DEVICE_ERROR_RATIO_SENSOR = "device_error_ratio_sensor"

# --- Define C++ class bindings ---
olimpia_bridge_ns = cg.esphome_ns.namespace("olimpia_bridge")
OlimpiaBridge = olimpia_bridge_ns.class_("OlimpiaBridge", cg.Component)
OlimpiaBridgeClimate = olimpia_bridge_ns.class_("OlimpiaBridgeClimate", climate.Climate, cg.Component)
ModbusAsciiHandler = olimpia_bridge_ns.class_("ModbusAsciiHandler", cg.Component)

# --- Per-climate configuration schema ---
olimpia_bridge_climate_schema = climate.climate_schema(OlimpiaBridgeClimate).extend({
    cv.Required(CONF_NAME): cv.string,
    cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=247),
    cv.Optional(CONF_EMA_ALPHA, default=0.2): cv.float_,
    cv.Optional(CONF_WATER_TEMPERATURE_SENSOR): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=0,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_MIN_TEMPERATURE, default=15.0): cv.float_,
    cv.Optional(CONF_MAX_TEMPERATURE, default=30.0): cv.float_,
    cv.Optional(CONF_TARGET_TEMPERATURE_STEP, default=0.5): cv.float_,
    cv.Optional(CONF_PRESETS_ENABLED, default=False): cv.boolean,
    cv.Optional(CONF_DISABLE_MODE_AUTO, default=False): cv.boolean,
    cv.Optional(CONF_DEVICE_ERROR_RATIO_SENSOR): sensor.sensor_schema(
        unit_of_measurement="%",
        entity_category="diagnostic",
    ),
})

# --- Top-level configuration schema ---
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(OlimpiaBridge),
    cv.GenerateID(CONF_HANDLER_ID): cv.declare_id(ModbusAsciiHandler),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required("re_pin"): pins.gpio_output_pin_schema,
    cv.Required("de_pin"): pins.gpio_output_pin_schema,
    cv.Required("error_ratio_sensor"): sensor.sensor_schema(
        unit_of_measurement="%",
        entity_category="diagnostic",
    ),
    cv.Required(CONF_CLIMATES): cv.ensure_list(olimpia_bridge_climate_schema),
    cv.Optional(CONF_USE_EMA, default=True): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

# --- Code generation logic ---
async def to_code(config):
    # First create and configure the Modbus handler
    handler = cg.new_Pvariable(config[CONF_HANDLER_ID])
    await cg.register_component(handler, config)

    # Configure hardware on handler
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    re_pin = await cg.gpio_pin_expression(config["re_pin"])
    de_pin = await cg.gpio_pin_expression(config["de_pin"])

    cg.add(handler.set_uart(uart_var))
    cg.add(handler.set_re_pin(re_pin))
    cg.add(handler.set_de_pin(de_pin))

    # Create bridge with pre-configured handler
    bridge = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(bridge, config)
    cg.add(bridge.set_handler(handler))

    # Process each climate entity in configuration
    for climate_conf in config[CONF_CLIMATES]:
        climate_var = cg.new_Pvariable(climate_conf[CONF_ID])
        await climate.register_climate(climate_var, climate_conf)
        await cg.register_component(climate_var, climate_conf)

        cg.add(climate_var.set_address(climate_conf[CONF_ADDRESS]))
        cg.add(climate_var.set_handler(handler))
        cg.add(bridge.add_climate(climate_var))
        cg.add(climate_var.set_ambient_ema_alpha(climate_conf[CONF_EMA_ALPHA]))
        cg.add(climate_var.set_use_ema(config[CONF_USE_EMA]))

        # Set temperature traits
        cg.add(climate_var.set_min_temperature(climate_conf[CONF_MIN_TEMPERATURE]))
        cg.add(climate_var.set_max_temperature(climate_conf[CONF_MAX_TEMPERATURE]))
        cg.add(climate_var.set_target_temperature_step(climate_conf[CONF_TARGET_TEMPERATURE_STEP]))

        # Optional water temperature sensor
        if CONF_WATER_TEMPERATURE_SENSOR in climate_conf:
            sens = await sensor.new_sensor(climate_conf[CONF_WATER_TEMPERATURE_SENSOR])
            cg.add(climate_var.set_water_temp_sensor(sens))

        # Optional per-climate error ratio sensor (now named device_error_ratio_sensor)
        if CONF_DEVICE_ERROR_RATIO_SENSOR in climate_conf:
            device_error_sensor = await sensor.new_sensor(climate_conf[CONF_DEVICE_ERROR_RATIO_SENSOR])
            cg.add(climate_var.set_error_ratio_sensor(device_error_sensor))

    # Configure error ratio sensor if present
    if "error_ratio_sensor" in config:
        error_sensor = await sensor.new_sensor(config["error_ratio_sensor"])
        cg.add(bridge.set_error_ratio_sensor(error_sensor))
