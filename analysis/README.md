# Project Overview

This repository contains tools, scripts, and simulations for calibrating, testing, and analyzing data from an embedded system, likely related to a flight controller or similar sensor-based project.

## File Descriptions

### Calibration & Analysis
* **`calib.py`**: See **`CALIBRATION.md`**
* **`parse.py`**: General script to handle the data files from a flight. If run with a filepath `python3 parse.py path/to/file.bin` it will plot the binary logs.

### Testing
* **`run_test.py`**: This can be used in conjuction with a test build from the main firmware to run a test over usb. It is currently not finished and requires inspection an modification of test the code (found in `sim_test.py`) to use. To currently run it is `python3 run_test.py sim_test.py /path/to/ACMboard` 
* **`sim_test.py`**: Contains the code that interfaces between the board and the SCRT python simulation. It can be modified to change the exact simulation that will be run.
  
### Miscellaneous
* **`kalman.py`**: Implements a Kalman Filter for state estimation (height and velocity). Includes `KalmanFilter` class for math and `FilterSim` for running simulations of the filter.
* **`kalman_simple.py`**: A simulation tool that uses `kalman.py` to test the Kalman Filter against various parameters (accel, bias, noise, etc.) and plot results.
* **`quat_int.py`**: A script used to verify quaternion rotation logic
