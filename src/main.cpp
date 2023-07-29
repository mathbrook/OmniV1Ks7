
#include "ShockPot Driver.h"
#include <Arduino.h>


void setup()
{
  	Serial.begin(9600);
}


void loop()
{
  	// Take reading of Shock Pots
	ShockPot_.setReading();

  	// Print readings to Serial
  	ShockPot_.getReading();
}

