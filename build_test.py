import os
import importlib
import pathlib

Import('env')

test_path = os.getenv('TEST_FILE', None)
assert test_path is not None, 'No test path provided'

test_gen = importlib.import_module(test_path)

def check_attr(attr, attr_type):
    assert hasattr(test_gen, attr), f'No {attr} defined'
    assert isinstance(getattr(test_gen, attr), int), f'{attr} must be {attr_type}'


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


acc_data = test_gen.get_acc()
check_arr(acc_data, 3, 'acc_data')
hg_acc_data = test_gen.get_hg_acc()
check_arr(hg_acc_data, 3, 'hg_acc_data')

gyro_data = test_gen.get_gyro()
check_arr(gyro_data, 3, 'gyro_data')

baro_data = test_gen.get_baro()
check_arr(baro_data, 2, 'baro_data')

with open(pathlib.Path('src', 'test_data.h')) as file:
    file.write(
f'''
max_steps = {test_gen.max_steps};
micros_per_step = {test_gen.micros_per_step};

acc_x_data = {{{', '.join(acc[0] for acc in acc_data)}}};
acc_y_data = {{{', '.join(acc[1] for acc in acc_data)}}};
acc_z_data = {{{', '.join(acc[2] for acc in acc_data)}}};

hg_acc_x_data = {{{', '.join(hg_acc[0] for hg_acc in hg_acc_data)}}};
hg_acc_y_data = {{{', '.join(hg_acc[1] for hg_acc in hg_acc_data)}}};
hg_acc_z_data = {{{', '.join(hg_acc[2] for hg_acc in hg_acc_data)}}};

gyro_x_data = {{{', '.join(gyro[0] for gyro in gyro_data)}}};
gyro_y_data = {{{', '.join(gyro[1] for gyro in gyro_data)}}};
gyro_z_data = {{{', '.join(gyro[2] for gyro in gyro_data)}}};

pres_data = {{{', '.join(baro[0] for baro in baro_data)}}};
temp_data = {{{', '.join(baro[1] for baro in baro_data)}}};
'''
    )

