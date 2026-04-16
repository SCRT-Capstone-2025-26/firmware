import sys
import importlib.util
import argparse
import pathlib
import parse
import tqdm

parser = argparse.ArgumentParser()
parser.add_argument('test_file')
parser.add_argument('test_id')
parser.add_argument('data_folder')

args = parser.parse_args()

test_path = pathlib.Path(args.test_file)
test_id = args.test_id
data_path = pathlib.Path(args.data_folder)

# See https://stackoverflow.com/questions/79852343/how-can-i-dynamically-load-and-execute-foo-py-if-it-contains-relative-imports
# I don't want to force the scripts to be rewritten so I do sys path
# It would probably be more correct to pop some stuff from sys path and then re add it
sys.path.append(str(test_path.parent))
spec = importlib.util.spec_from_file_location('test_gen', test_path)
test_gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(test_gen)
sys.path.pop()

assert hasattr(test_gen, 'check_sample'), 'check_sample must exist in the test'

all_log_files = list(pathlib.Path(data_path, 'Logs').iterdir())
test_log_files = []
for path in all_log_files:
    name = path.name
    curr_test_id = name.removeprefix('log_test_').split('_')[-2]
    if test_id != curr_test_id:
        continue

    test_log_files.append(path)

# The last file is ignored since it may be malformed due to the being uplugged while running it
bar = tqdm.tqdm(test_log_files[:-1], unit='file')
for path in bar:
    name = path.name
    if not name.startswith('log_test_'):
        continue

    index = int(name.split('_')[-1].removesuffix('.txt'))

    curr_test_id = name.removeprefix('log_test_').removesuffix(f'_{index}.txt')
    if test_id != curr_test_id:
        continue

    with open(path, 'r') as file:
        log = parse.read_logs(file)

    data_file_path = pathlib.Path(data_path, 'Data', f'data_test_{test_id}_{index}.bin')
    with open(data_file_path, 'rb') as file:
        data = parse.read_all(file)

    res, error = test_gen.check_sample(log, data)
    if not res:
        bar.write(f'File {index} failed with {error}')

bar.close()

