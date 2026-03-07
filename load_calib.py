try:
    import yaml
except ModuleNotFoundError:
    print('Please install PyYAML or requirements.txt')
    exit(1)
import os

Import('env')

default_name = 'default'
board_name = os.getenv('BOARD', default_name)

config_file = os.path.join('calibrations', f'{board_name}.yaml')
default_file = os.path.join('calibrations', f'{default_name}.yaml')

# NOTE: This script is only designed to handle numbers, booleans, and arrays of numbers
if os.path.exists(config_file):
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    with open(default_file, 'r') as f:
        default_config = yaml.safe_load(f)

    # Check that the calibration file is the same as the default file with different data
    # NOTE: Doesn't check list's sub values since all arrays are assumed to be numbers
    assert len(default_config.keys()) == len(config.keys())
    for key, value in config.items():
        assert type(default_config[key]) is type(value)

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
