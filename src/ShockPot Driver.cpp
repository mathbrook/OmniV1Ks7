

#include "ShockPot Driver.h"
#include <Arduino.h>




void ShockPot::takeReading()
{
    this->pot_1.val = analogRead(A0);
    delay(10);
    this->pot_2.val = analogRead(A1);
    this->pot_3.val = analogRead(A2);
    this->pot_4.val = analogRead(A3);
}
