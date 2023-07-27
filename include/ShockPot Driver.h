#pragma once

#ifndef SHOCKPOT_DRIVER.H
#define SHOCKPOT_DRIVER.H 


typedef struct 
{
    int val = 0;
    int pos = 0;
} ShockPot;


class ShockPotDriver
{
private:
    ShockPot pot[4];
public:
    void setReading();
    int  getReading();
    void getPosition();
};

static ShockPotDriver ShockPot_;


#endif