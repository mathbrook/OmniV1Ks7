#pragma once

#ifndef SHOCKPOT_DRIVER.H
#define SHOCKPOT_DRIVER.H 


#include <Arduino.h>


const static uint8_t ShockPotPins[] = {A0, A1, A2, A3};

typedef struct 
{
	uint8_t  pin = 0;
    uint16_t val = 0;
    uint16_t pos = 0;
} ShockPot;


class ShockPotDriver
{
private:
    ShockPot pot[4];
public:
	ShockPotDriver();
    ~ShockPotDriver();
    void setReading();
    int  getReading();
    void getPosition();
};


static ShockPotDriver ShockPot_;


#endif

