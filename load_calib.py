import tomllib
import pathlib
import os

Import('env')

DEFAULT_NAME = 'default'
board_name = os.getenv('BOARD', DEFAULT_NAME)

config_file = pathlib.Path('calibrations', f'{board_name}.toml')
default_file = pathlib.Path('calibrations', f'{DEFAULT_NAME}.toml')

# NOTE: This script is only designed to handle numbers, booleans, and arrays of numbers
if os.path.exists(config_file):
    with open(config_file, 'rb') as file:
        config = tomllib.load(file)

    print(config)

    with open(default_file, 'rb') as file:
        default_config = tomllib.load(file)

    # Check that the calibration file is the same as the default file with different data
    # NOTE: Doesn't check list's sub values since all arrays are assumed to be numbers
    assert default_config.keys() >= config.keys(), 'Extra keys in config'
    assert default_config.keys() <= config.keys(), 'Missing keys in config'
    for key, value in config.items():
        default_type = type(default_config[key])
        value_type = type(value)
        assert default_type == value_type, f'Type mismatch for key {key}: expected {default_type}, got {value_type}'

    # Load the calibration into C++ defines
    defines = []
    for key, value in config.items():
        key = f'_CALIB_{key}'

        if isinstance(value, list):
            for i, sub_value in enumerate(value):
                defines.append((f'{key}_{i + 1}', str(sub_value)))
        else:
            # C++ doesn't capitalize its booleans
            if isinstance(value, bool):
                value = str(value).lower()

            defines.append((key, str(value)))

    print('Adding calibration')
    env.Append(CPPDEFINES=defines)
else:
    print(f'{config_file} not found')
