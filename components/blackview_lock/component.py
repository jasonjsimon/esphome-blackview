import esphome.config_validation as cv

# The simplest possible schema to prove the file is being loaded.
CONFIG_SCHEMA = cv.Schema({})

async def to_code(config):
    # This function does nothing for the test.
    pass