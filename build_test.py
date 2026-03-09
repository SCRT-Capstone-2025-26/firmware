import os
import importlib
import pathlib

Import('env')

test_id = os.getenv('TEST_ID', None)
assert test_id is not None, 'No test ID not provided'
# To make sure that this won't cause escaping problems
assert test_id.isdigit(), 'Test ID contains no digit characters'

# Pio doesn't seem to properly escape the c++ defines so we have to escape the double quotes
env.Append(CPPDEFINES=[
  ('TEST_ID', f'\\"{test_id}\\"')
])

test_path = os.getenv('TEST_FILE', None)
assert test_path is not None, 'No test path provided'

spec = importlib.util.spec_from_file_location('test_gen', test_path)
test_gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(test_gen)

def check_attr(attr, attr_type):
    assert hasattr(test_gen, attr), f'No {attr} defined'
    assert isinstance(getattr(test_gen, attr), attr_type), f'{attr} must be {attr_type}'


check_attr('max_steps', int)
check_attr('micros_per_step', int)

check_attr('acc_data', list)
check_attr('hg_acc_data', list)

check_attr('gyro_data', list)

check_attr('baro_data', list)

def check_arr(arr, entry_size, name):
    assert len(arr) == test_gen.max_steps, f'{name} must have length max_steps'
    assert all(len(x) == entry_size for x in arr), f'{name} must contain tuples of length {entry_size}'
    assert all(all(isinstance(value, float) for value in x) for x in arr), f'{name} must contain only tuples of floats'


acc_data = test_gen.acc_data
check_arr(acc_data, 3, 'acc_data')
hg_acc_data = test_gen.hg_acc_data
check_arr(hg_acc_data, 3, 'hg_acc_data')

gyro_data = test_gen.gyro_data
check_arr(gyro_data, 3, 'gyro_data')

baro_data = test_gen.baro_data
check_arr(baro_data, 2, 'baro_data')

with open(pathlib.Path('src', 'test_data.h'), 'w+') as file:
    file.write(
f'''\
size_t max_steps = {test_gen.max_steps};
size_t micros_per_step = {test_gen.micros_per_step};

float acc_x_data[] = {{{', '.join(str(acc[0]) for acc in acc_data)}}};
float acc_y_data[] = {{{', '.join(str(acc[1]) for acc in acc_data)}}};
float acc_z_data[] = {{{', '.join(str(acc[2]) for acc in acc_data)}}};

float hg_acc_x_data[] = {{{', '.join(str(hg_acc[0]) for hg_acc in hg_acc_data)}}};
float hg_acc_y_data[] = {{{', '.join(str(hg_acc[1]) for hg_acc in hg_acc_data)}}};
float hg_acc_z_data[] = {{{', '.join(str(hg_acc[2]) for hg_acc in hg_acc_data)}}};

float gyro_x_data[] = {{{', '.join(str(gyro[0]) for gyro in gyro_data)}}};
float gyro_y_data[] = {{{', '.join(str(gyro[1]) for gyro in gyro_data)}}};
float gyro_z_data[] = {{{', '.join(str(gyro[2]) for gyro in gyro_data)}}};

float pres_data[] = {{{', '.join(str(baro[0]) for baro in baro_data)}}};
float temp_data[] = {{{', '.join(str(baro[1]) for baro in baro_data)}}};

'''
    )

