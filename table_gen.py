import numpy as np
import pathlib

ext_table = np.load('ext_table.h')
# (height, vel) so row major
table = np.load('table.h')



# TODO: height start and step should be loaded
height_start = 0.0
height_step = 0.0

with open(pathlib.Path('src', 'table_data.h'), 'w+') as out_file:
    out_file.write(f'''\
#include "table.h"

const float TABLE[] = {{{', '.join(table.flatten('C'))}}};

const float ROW_WEIGHTS[] = {{{', '.join(ext_table)}}}

const float TABLE_SIZE = {len(table)};
const float ROW_SIZE = {len(ext_table)};

const float HEIGHT_START = {height_start};
const float HEIGHT_STEP = {height_step};
''')

