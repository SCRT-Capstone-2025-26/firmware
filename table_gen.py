import numpy as np
import pathlib

height_table = np.load('heights.npy')
ext_table = np.load('exts.npy')
# (height, vel)
table = np.load('table.npy')

# Check that height table and extension table are monotonically increasing
# Which is assumed in the table lookup
assert np.all(np.diff(height_table) > 0)
assert np.all(np.diff(ext_table) > 0)

# Make sure the arrays are properly sized if they are
# size < 2 then the binary search fails
assert len(height_table) >= 2
assert len(ext_table) >= 2
assert table.shape[0] == len(height_table)
assert table.shape[1] == len(ext_table)

with open(pathlib.Path('src', 'table_data.h'), 'w+') as out_file:
    out_file.write(f'''\
#include "table.h"

const float TABLE[] = {{{', '.join(table.flatten('C'))}}};

const float HEIGHTS[] = {{{', '.join(height_table)}}}
const size_t HEIGHTS_SIZE = {{{len(height_table)}}}

const float EXTENSIONS[] = {{{', '.join(ext_table)}}}
const size_t EXTENSIONS_SIZE = {{{len(height_table)}}}
''')

