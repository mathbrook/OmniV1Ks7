

#include "ShockPot Driver.h"
#include <SD.h>
#include <Arduino.h>




void ShockPotDriver::setReading()
{
    for (int i = 0; i < sizeof(this->pot); i++)
    {
        this->pot[i].val = analogRead(A0 + i);
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
    // Very advace complicated math goes here
    // Like calc and shit dawg
}
