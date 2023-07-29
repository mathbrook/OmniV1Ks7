

#include "ShockPot Driver.h"
#include <SD.h>
#include <Arduino.h>



ShockPotDriver::ShockPotDriver()
{
    for (int i = 0; i < sizeof(this->pot); i++)
    {
        this->pot[i].pin = ShockPotPins[i];
    }
}


ShockPotDriver::~ShockPotDriver()
{

}


void ShockPotDriver::setReading()
{
    for (int i = 0; i < sizeof(this->pot); i++)
    {
        this->pot[i].val = analogRead(this->pot[i].pin);
        delay(10);
    }

    this->getPosition();
}


int ShockPotDriver::getReading()
{
    for (int i = 0; i < sizeof(this->pot); i++)
    {
        Serial.print("Shock Pot ");
        Serial.print(i + ": ");
        Serial.println(this->pot[i].val);
    }

    return 0;
}


void ShockPotDriver::getPosition()
{
    Serial.println("SIKE!");
    // Very advanced complicated math goes here
    // Like calc and shit dawg
}

