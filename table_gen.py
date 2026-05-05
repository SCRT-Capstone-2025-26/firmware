import numpy as np
import pathlib

# NOTE: This could be modified to only run when needed

heights = np.load("tables/heights.npy")
# Angles is in radians, but the table is index by cos zenith
# It is reversed because the cos
angles = np.cos(np.load("tables/angles.npy"))[::-1]
exts = np.load("tables/exts.npy")
# (height, angle, vel)
# Reverse to match the angle
lookup = np.load("tables/lookup.npy")[:, ::-1]

# Check that height array and extension array are monotonically increasing
# Which is assumed in the lookup lookup
assert np.all(np.diff(heights) > 0)
assert np.all(np.diff(angles) > 0)
assert np.all(np.diff(exts) > 0)
# We also require every row is monotonically increasing as well
# Little numerical trick to achieve this since the table is not
#  generated to be montonic
# TODO: Fix this somehow
lookup += np.linspace(-0.0001, 0.0001, len(exts))
assert np.all(np.diff(lookup) > 0)

# Make sure the arrays are properly sized if they are
# size < 2 then the binary search fails
assert len(heights) >= 2
assert len(angles) >= 2
assert len(exts) >= 2
assert lookup.shape[0] == len(heights)
assert lookup.shape[1] == len(angles)
assert lookup.shape[2] == len(exts)

print(heights, exts, angles)
print(lookup[4, 6])

# Convert array to C array hex floats are used to maintain precision
def arr_to_cont(arr):
    arr = np.asarray(arr, dtype=np.float64)
    return ", ".join([x.hex() for x in arr])


with open(pathlib.Path("src", "table_data.h"), "w+") as out_file:
    out_file.write(f"""\
#include "table.h"

const float LOOKUP[] = {{{arr_to_cont(lookup.flatten('C'))}}};

const float HEIGHTS[] = {{{arr_to_cont(heights)}}};
const size_t HEIGHTS_SIZE = {len(heights)};

const float ANGLES[] = {{{arr_to_cont(angles)}}};
const size_t ANGLES_SIZE = {len(angles)};

const float EXTENSIONS[] = {{{arr_to_cont(exts)}}};
const size_t EXTENSIONS_SIZE = {len(exts)};
""")
