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

## Hardware

See the hardware [repo](https://github.com/osu-asdt/beavs-6) for more details.

The main chip is the [rp2040](https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf?disposition=inline) and we use the Arduino base, but also make use of many hardware specific functions.

Currently we use the imu [ISM6HG256x](https://www.st.com/resource/en/datasheet/ism6hg256x.pdf), barometer [MS5607](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5607-02BA03%7FB%7Fpdf%7FEnglish%7FENG_DS_MS5607-02BA03_B.pdf%7FCAT-BLPS0035) (which is compatible with the MS5611 standard), a PWM servo (no datasheet since this is not set in stone), and an (unimplemented) [radio]().

We may end up using the [GPS](), but currently do not. All other sensors are not used.

## Software Outline

This software is centered around two main loops. Each running on a separate core. One is the logging core which logs data to the SD and Serial outputs and the other has the main control loop. The logging core allows the main control loop to run without any significant delays.

This code is designed to be robust and handle a power failure or watchdog event in flight. This significantly increases the software's complexity.

The main control loops runs differently depending on which state it is in.

- **BOOT**: The first mode entered. Where all sensors are inited.
- **UNKNOWN**: Where it waits a few to detecting whether it is reading data consistent with flight and if so jumps to the flight state
- **Unarmed**: An idle mode waiting for the arm switch (it records imu data in this mode)
- **Armed**: An idle mode where it waits and records imu data waiting for a consistent launch signal from the imu
- **Flight**: The board is in flight and controls are active. On the transition to flight from **Armed** the stored imu data is used to calibrate the angle and then playback the launch history. If launch from **UNKNOWN** it reads old data from flash to estimate its position. In this mode the barometer is also active providing an alternate source of information about the height and both barometer and imu are fed into the Kalman filter. In this mode it also stores its state in flash in case of a reboot. Finally this mode also engages active controls when the imu reads a safe acceleration for active controls. This mode finishes when the angle is 30 degrees to be compliant with competition rules.
- **Done**: The flight has been finished and the board should retract the blades and idle.

## Team

This was created for an OSU capstone and more info can be found at the main [repo](https://github.com/SCRT-Capstone-2025-26/SCRT_Rocket_SIM).
