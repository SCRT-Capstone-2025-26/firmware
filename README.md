# Firmware for the BEAVS 2026 active flight control

## Overview

This is the repo for SCRT's beavs module. Which is designed to extend servo
actuated blades to control rocket drag and hit a target height of 10,000 ft.
This contains the arduino firmware (in Arduino/BEAVS5_Main) for the system as
well as a C++ simulation to run the firmware in a variety of test conditions.
This simulation has python bindings through pybind11 to allow for usage a python
rocket simulation [here](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).

This board is undergoing hardware redesign so this code is in flux.

## How Run

This firmware uses a platformio build system which can be installed [here](https://platformio.org/).

This can be run on a BEAVS 2026 board through ``pio run -e release -t upload`` and in debug mode through ``pio run -e debug -t upload`` once you have platformio setup.

## Team

This was created for an OSU capstone and more info can be found at the main [repo](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).
