import csv
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("in_path")
parser.add_argument("out_path")
args = parser.parse_args()

if os.path.getmtime(args.in_path) < os.path.getmtime(args.out_path):
    exit(0)

with open(args.in_path, 'r') as in_file:
    reader = csv.reader(in_file)

    height_start, height_step, *row_values = next(reader)

    values = []
    for row in reader:
        values.extend(row)


with open(args.out_path, 'w+') as out_file:
    out_file.write('#include "table.h"\n\n')

    out_file.write('const float TABLE[] = { ')
    out_file.write(', '.join(values))
    out_file.write(' };\n')

    out_file.write('const float ROW_WEIGHTS[] = { ')
    out_file.write(', '.join(row_values))
    out_file.write(' };\n\n')

    out_file.write(f'const float TABLE_SIZE = {len(values)};\n')
    out_file.write(f'const float ROW_SIZE = {len(row_values)};\n\n')

    out_file.write(f'const float HEIGHT_START = {height_start};\n')
    out_file.write(f'const float HEIGHT_STEP = {height_step};\n')

