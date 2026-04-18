import numpy as np
import pathlib

# NOTE: This could be modified to only run when needed

heights = np.load('tables/heights.npy')
exts = np.load('tables/exts.npy')
# (height, vel)
lookup = np.load('tables/lookup.npy')

# Check that height array and extension array are monotonically increasing
# Which is assumed in the lookup lookup
assert np.all(np.diff(heights) > 0)
assert np.all(np.diff(exts) > 0)

# Make sure the arrays are properly sized if they are
# size < 2 then the binary search fails
assert len(heights) >= 2
assert len(exts) >= 2
assert lookup.shape[0] == len(heights)
assert lookup.shape[1] == len(exts)

def arr_to_cont(arr):
    arr = np.asarray(arr, dtype=np.float64)
    return ', '.join([x.hex() for x in arr])

with open(pathlib.Path('src', 'table_data.h'), 'w+') as out_file:
    out_file.write(f'''\
#include "table.h"

const float LOOKUP[] = {{{arr_to_cont(lookup.flatten('C'))}}};

const float HEIGHTS[] = {{{arr_to_cont(heights)}}};
const size_t HEIGHTS_SIZE = {len(heights)};

const float EXTENSIONS[] = {{{arr_to_cont(exts)}}};
const size_t EXTENSIONS_SIZE = {len(exts)};

''')

