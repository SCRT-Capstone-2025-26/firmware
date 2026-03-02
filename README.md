# SCRT BEAVS 6 (2026) Firmware

## Overview

This is the repo for SCRT's [BEAVS 6 module](https://github.com/osu-asdt/beavs-6) firmware.
BEAVS is designed to extend servo actuated blades to control rocket drag and hit a
target height of 10,000 ft.

## How to run

This firmware uses a platformio build system which can be installed [here](https://platformio.org/).

This can be run on a BEAVS 2026 board through ``pio run -e release -t upload`` and in debug mode through ``pio run -e debug -t upload`` once you have platformio setup.


### Linux – D-Bus rules

The following rules have been tested to work for flashing the BEAVS 6 board over USB.
Two entries are necessary to cover the board when running the BEAVS firmware as well as in
bootloader mode.

These rules also assume your user is in the `plugdev` group.

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", GROUP="plugdev", MODE:="0660"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", GROUP="plugdev", MODE:="0660"
```

## Team

This was created for an OSU capstone and more info can be found at the main [repo](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).
