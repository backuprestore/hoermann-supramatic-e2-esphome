import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from . import garage_door_ns, GarageDoorComponent, CONF_GARAGE_DOOR_ID

GarageDoorCover = garage_door_ns.class_("GarageDoorCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = (
    cover.cover_schema(GarageDoorCover)
    .extend(
        {
            cv.GenerateID(CONF_GARAGE_DOOR_ID): cv.use_id(GarageDoorComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_GARAGE_DOOR_ID])
    cg.add(var.set_parent(parent))
    cg.add(parent.set_cover(var))
