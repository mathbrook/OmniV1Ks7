#pragma once

#ifndef SHOCKPOT_DRIVER.H
#define SHOCKPOT_DRIVER.H 

const static uint8_t ShockPotPins[] = {A0, A1, A2, A3};

typedef struct 
{
	int pin = 0;
    int val = 0;
    int pos = 0;
} ShockPot;


class ShockPotDriver
{
private:
    ShockPot pot[4];
public:
	ShockPotDriver();
    void setReading();
    int  getReading();
    void getPosition();
};

static ShockPotDriver ShockPot_;


#endif