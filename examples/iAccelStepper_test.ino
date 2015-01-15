//#include <Energia.h>

#include "AccelStepper.h"
#include "iAccelStepper.h"

void dummy(void) {};

iAccelStepper axis1;
iAccelStepper axis2;

void setup()
{
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  Serial.begin(115200);
  axis1.begin(PB_0, PB_5);
  axis2.begin(PE_4, PB_1);
  Serial.println("instantiated");
  axis1.setMaxSpeed(20000);
  axis2.setMaxSpeed(20000);
//  axis1.setAcceleration(150000);
  axis1.setAcceleration(15000);
  axis2.setAcceleration(13000);
  Serial.println("configured");
}

unsigned long prev, loopc;

void loop()
{
  ++loopc;
  if(digitalRead(PUSH1) == 0) {
    axis1.move(30000);
  } else {
    axis1.stop();
  }

  if(digitalRead(PUSH2) == 0) {
    axis2.move(30000);
  } else {
    axis2.stop();
  }
  if((millis() - prev) > 1000) {
    Serial.print("\nloop counter:\t");
    Serial.println(loopc, DEC);
    loopc = 0;
    Serial.print("interval:\t");
    Serial.print(axis1._stepInterval, DEC);
    Serial.print("\t: ");
    Serial.println(axis2._stepInterval, DEC);
    Serial.print("state:\t");
    Serial.print(axis1.run()?"true":"false");
    Serial.print("\t: ");
    Serial.println(axis2.run()?"true":"false");
    Serial.print("speed:\t");
    Serial.print(axis1.speed(), 3);
    Serial.print("\t: ");
    Serial.println(axis2.speed(), 3);
    Serial.print("position:\t");
    Serial.print(axis1.currentPosition(), DEC);
    Serial.print("\t: ");
    Serial.println(axis2.currentPosition(), DEC);
    Serial.print("distance:\t");
    Serial.print(axis1.distanceToGo(), DEC);
    Serial.print("\t: ");
    Serial.println(axis2.distanceToGo(), DEC);
    prev = millis();
  }
  digitalWrite(RED_LED, 0);
}



