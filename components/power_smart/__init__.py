import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import cc1101
from esphome.components import remote_transmitter
from esphome.const import CONF_ID

CODEOWNERS = ["@user"]
DEPENDENCIES = ["cc1101", "remote_transmitter"]
MULTI_CONF = True

power_smart_ns = cg.esphome_ns.namespace("power_smart")
PowerSmartComponent = power_smart_ns.class_("PowerSmartComponent", cg.Component)
PowerSmartCommand = power_smart_ns.enum("PowerSmartCommand")

POWER_SMART_COMMANDS = {
    "UP": PowerSmartCommand.POWER_SMART_COMMAND_UP,
    "DOWN": PowerSmartCommand.POWER_SMART_COMMAND_DOWN,
    "STOP": PowerSmartCommand.POWER_SMART_COMMAND_STOP,
}

CONF_CC1101_ID = "cc1101_id"
CONF_REMOTE_TRANSMITTER_ID = "remote_transmitter_id"
CONF_REPEAT = "repeat"

CONF_REMOTE_ID = "remote_id"
CONF_CHANNEL = "channel"
CONF_COMMAND = "command"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PowerSmartComponent),
        cv.GenerateID(CONF_CC1101_ID): cv.use_id(cc1101.CC1101Component),
        cv.GenerateID(CONF_REMOTE_TRANSMITTER_ID): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
        cv.Optional(CONF_REPEAT, default=10): cv.int_range(min=1, max=50),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    radio = await cg.get_variable(config[CONF_CC1101_ID])
    cg.add(var.set_cc1101(radio))

    transmitter = await cg.get_variable(config[CONF_REMOTE_TRANSMITTER_ID])
    cg.add(var.set_remote_transmitter(transmitter))

    cg.add(var.set_repeat(config[CONF_REPEAT]))


POWER_SMART_SEND_COMMAND_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(PowerSmartComponent),
        cv.Required(CONF_REMOTE_ID): cv.templatable(cv.hex_uint16_t),
        cv.Required(CONF_CHANNEL): cv.templatable(cv.int_range(min=1, max=6)),
        cv.Required(CONF_COMMAND): cv.templatable(cv.enum(POWER_SMART_COMMANDS, upper=True)),
    }
)


@automation.register_action(
    "power_smart.send_command",
    power_smart_ns.class_("PowerSmartSendCommandAction", automation.Action),
    POWER_SMART_SEND_COMMAND_SCHEMA,
    synchronous=True,
)
async def power_smart_send_command_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)

    template_ = await cg.templatable(config[CONF_REMOTE_ID], args, cg.uint16)
    cg.add(var.set_remote_id(template_))

    template_ = await cg.templatable(config[CONF_CHANNEL], args, cg.uint8)
    cg.add(var.set_channel(template_))

    template_ = await cg.templatable(config[CONF_COMMAND], args, PowerSmartCommand)
    cg.add(var.set_command(template_))

    return var
