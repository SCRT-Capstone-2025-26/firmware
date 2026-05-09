import argparse as ap
import sys
import importlib.util
import pathlib

import test_comm


if __name__ == '__main__':
    parser = ap.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("test")
    args = parser.parse_args()

    # See https://stackoverflow.com/questions/79852343/how-can-i-dynamically-load-and-execute-foo-py-if-it-contains-relative-imports
    # I don't want to force the scripts to be rewritten so I do sys path
    # It would probably be more correct to pop some stuff from sys path and then re add it
    test_path = pathlib.Path(args.test)
    sys.path.append(str(test_path.parent))
    spec = importlib.util.spec_from_file_location("test_run", test_path)
    test_run = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(test_run)
    sys.path.pop()

    # TODO: A with statement should be used
    # TODO: Fix noise
    dm = test_comm.DataManager(args.port)
    dm.start()
    dm.ser.write(b"Test 1\0")

    test_run.run_test(dm)

    dm.stop(None, None, None)
