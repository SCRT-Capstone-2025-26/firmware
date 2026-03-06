#ifndef INA745_H
#define INA745_H

#include <Arduino.h>
#include <Wire.h>

enum SampleMode {
    SHUTDOWN = 0x0,
    TRIGGERED_BUS = 0x1,
    TRIGGERED_TEMP = 0x4,
    TRIGGERED_TEMP_VOLT = 0x5,
    TRIGGERED_TEMP_CURR = 0x6,
    TRIGGERED_TEMP_CURR_VOLT = 0x7,
    CONTINUOUS_VOLT = 0x9,
    CONTINUOUS_TEMP = 0xc,
    CONTINUOUS_TEMP_VOLT = 0xd,
    CONTINUOUS_TEMP_CURR = 0xe,
    CONTINUOUS_TEMP_CURR_VOLT = 0xf
};

enum ConversionTime {
    T_50us = 0x0,
    T_84us = 0x1,
    T_150us = 0x2,
    T_280us = 0x3,
    T_540us = 0x4,
    T_1052us = 0x5,
    T_2074us = 0x6,
    T_4120us = 0x7
};

enum AverageCount {
    A_1 = 0x0,
    A_4 = 0x1,
    A_16 = 0x2,
    A_64 = 0x3,
    A_128 = 0x4,
    A_256 = 0x5,
    A_512 = 0x6,
    A_1024 = 0x7
};

enum INA745Result {
    INA_SUCCESS = 0,
    INA_NOT_DET = 1
};

class INA745 {
    uint8_t address;
    uint16_t voltage;
    int16_t temp;
    int16_t current;
    uint32_t power;
    uint64_t energy;
    uint64_t charge;
    TwoWire* wire;

    //INA745Result set_sample_mode(SampleMode mode);
    //INA745Result set_conversion_time(ConversionTime time);
    //INA745Result set_averaging_count(AverageCount count);

    INA745Result read_reg(uint8_t reg, uint8_t out[], int size);
    INA745Result read_reg(uint8_t reg, uint8_t &out);
    INA745Result read_reg(uint8_t reg, uint16_t &out);

    INA745Result write_reg(uint8_t reg, uint8_t in[], int size);
    INA745Result write_reg(uint8_t reg, uint8_t in);
    INA745Result write_reg(uint8_t reg, uint16_t in);

public:
    INA745(uint8_t address, TwoWire* wire);

    INA745Result begin();

    INA745Result reset();

    INA745Result read();

    uint16_t bus_millivolts();
    int32_t temperature_millicelsius();
    int32_t current_milliamps();
    uint32_t power_microwatts();
};

#endif
