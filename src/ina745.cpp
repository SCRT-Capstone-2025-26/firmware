#include <ina745.h>

INA745Result INA745::read_reg(uint8_t reg, uint8_t out[], int size) {
    wire->beginTransmission(address);
    wire->write(reg);
    wire->endTransmission();

    wire->requestFrom(address, size);
    for (int i = 0; i < size; i++) {
        int v = wire->read();
        out[i] = v;
    }

    return INA745Result::INA_SUCCESS;
}

INA745Result INA745::read_reg(uint8_t reg, uint8_t &out) {
    return read_reg(reg, &out, 1);
}

INA745Result INA745::read_reg(uint8_t reg, uint16_t &out) {
    auto res = read_reg(reg, (uint8_t*) &out, 2);
    out = (out >> 8) | (out << 8);
    return res;
}

INA745Result INA745::write_reg(uint8_t reg, uint8_t in[], int size) {
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(in, size);
    wire->endTransmission();

    return INA745Result::INA_SUCCESS;
}

INA745Result INA745::write_reg(uint8_t reg, uint8_t in) {
    return write_reg(reg, &in, 1);
}

INA745Result INA745::write_reg(uint8_t reg, uint16_t in) {
    uint16_t v = (in >> 8) | (in << 8);
    return write_reg(reg, (uint8_t*) &v, 2);
}

INA745Result INA745::begin() {
    reset();
    uint16_t res;
    read_reg(0x01, res);
    if (res != 0xfb68) return INA745Result::INA_NOT_DET;
    return INA745Result::INA_SUCCESS;
}

INA745Result INA745::reset() {
    return write_reg(0x00, (uint16_t)0x8000);
}

INA745::INA745(uint8_t address, TwoWire* wire) {
    this->address = address;
    this->wire = wire;
}

INA745Result INA745::read() {
    read_reg(0x05, voltage);
    read_reg(0x06, *((uint16_t*) &temp));
    temp >>= 4;
    read_reg(0x07, *((uint16_t*) &current));

    uint8_t buf[3];
    read_reg(0x08, buf, 3);
    power = ((uint32_t) buf[0] << 16) | ((uint32_t) buf[1] << 8) | ((uint32_t) buf[2]);

    return INA745Result::INA_SUCCESS;
}

uint16_t INA745::bus_millivolts() {
    return (voltage * 3) + (voltage / 8);
}

int32_t INA745::temperature_millicelsius() {
    return ((int32_t) temp) * 125;
}

int32_t INA745::current_milliamps() {
    return ((int32_t) current) + ((int32_t)current / 5);
}

uint32_t INA745::power_microwatts() {
    return power * 240;
}

