# How To Calibrate

## Collecting data

To collect calibration data first you have to plug in an SD card for the data to be written to then to calibrate the accelerometer you have to run the board in ``calib_lg`` and ``calib_hg`` with ``pio run -e [MODE] -t upload``. This will force the board to use the either low g or high g accelerometer.

To collect data that can be used to calibrate the accelerometer leave the board in at rest for a few seconds at various angles. Two different angles should be enough.

## Generating a calibration

Once you have collected the data you should take out the SD card which will have files corresponding to each run in sequential order in ``Data`` and ``Logs``. (The board should log the file id it used in each run).

Once you have identified the file in data corresponding to your run you can run ``python3 analysis/calib.py [FILE]`` to start the calibration. It will display a graph of the accelerometer readings and prompt you for ``How many steady regions``. This is the number of regions in the data where the board was at rest that you wish to use for calibration. It will then prompt for the start and end indices for each region. This refers to the index where the region starts and can be identified on the graph the of the data. When selecting steady regions the graph must look flat for each axis and the regions can't overlap.

Once all the values are inputted it will output a gyro bias, accelerometer bias and the value of little g that the accelerometer observed. This last value can be used to determine if the sensor is giving reasonable values.
