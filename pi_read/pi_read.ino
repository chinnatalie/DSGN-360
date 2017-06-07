#include <OrangutanDigital.h>
 /*
  * digital1: for the Orangutan controllers and 3pi robot
  *
  * This example uses the OrangutanDigital functions to read a digital
  * input and set a digital output.  It takes a reading on pin PC1, and
  * provides feedback about the reading on pin PD1 (the red LED pin).
  * If you connect a wire between PC1 and ground, you should see the
  * red LED change state.
  *
  * http://www.pololu.com/docs/0J20
  * http://www.pololu.com
  * http://forum.pololu.com
  */

void setup() {
  OrangutanDigital::setInput(IO_D0, PULL_UP_ENABLED);
  }

void loop() {
  if(OrangutanDigital::isInputHigh(IO_D0))     // Take digital reading of PC1.
    OrangutanDigital::setOutput(IO_D1, HIGH); // PC1 is high, so drive PD1 high.
  else{ 
    OrangutanDigital::setOutput(IO_D1, LOW);
  }  // PC1 is low, so drive PD1 low.
}


