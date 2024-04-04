#ifndef PINDEFS_H
#define PINDEFS_H
#include <stdint.h>
enum pinModes
{
    input = 0,
    output,
    input_pullup,
    input_pulldown,
    output_opendrain,
    input_disable
};
enum adcRes
{
    ADC_8BIT = 8,
    ADC_10BIT = 10,
    ADC_12BIT = 12
};
// Shock pot analog read pins
const uint8_t frpin = 17;
const uint8_t flpin = 16;
const uint8_t rrpin = 15;
const uint8_t rlpin = 14;
// Steering angle sensor
const uint8_t steeringpin = 21;

// All input pins:
const uint8_t input_pins[] = {
    frpin,
    flpin,
    rrpin,
    rlpin,
    steeringpin};

// RGB LED pins

uint8_t LED_RED = 6;
uint8_t LED_GREEN = 7;
uint8_t LED_BLUE = 8;
uint8_t led_builtin = 13;

const uint8_t output_pins[] = {
    LED_RED,
    LED_BLUE,
    LED_GREEN,
    led_builtin
};

void init_gpios(void (*initFunc)(uint8_t pin, uint8_t mode),  pinModes pinmode, const uint8_t *pins, int num_pins)
{
    for (int i = 0; i < num_pins; i++)
    {
        initFunc(pins[i], static_cast<uint8_t>(pinmode));
    }
};

void init_inputs(void (*initFunc)(uint8_t pin, uint8_t mode), void (*configFunc)(unsigned int res), adcRes resolution)
{
    init_gpios(initFunc, pinModes::input, input_pins, sizeof(input_pins) / sizeof(input_pins[0]));
    configFunc(static_cast<uint8_t>(resolution));
};

void init_outputs(void (*initFunc)(uint8_t pin, uint8_t mode))
{
    init_gpios(initFunc, pinModes::output, output_pins, sizeof(output_pins) / sizeof(output_pins[0]));

};
#endif
