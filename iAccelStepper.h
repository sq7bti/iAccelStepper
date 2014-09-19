#ifndef iAccelStepper_h
#define iAccelStepper_h

#include <Energia.h>
#include "AccelStepper.h"

#include "driverlib/sysctl.h"

#define MAX_INST 2

class iAccelStepper : public AccelStepper
{
public:
  iAccelStepper() : AccelStepper() {};
  void begin(uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4);
  void moveTo(long absolute);
  void move(long relative);
  boolean run(void) { return running; };
  unsigned long stepInterval() { return _stepInterval; };

  friend void timerISR0();
  friend void timerISR1();
//  friend void timerISR2();

protected:
  void ISR(void);

private:
  volatile boolean running;
  unsigned int id;
};

#endif
