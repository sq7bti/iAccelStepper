#ifndef iAccelStepper_h
#define iAccelStepper_h

#include <Energia.h>
#include "AccelStepper.h"

#include "driverlib/sysctl.h"

//*****************************************************************************
//
// The list of Timer peripherals.
//
//*****************************************************************************
static const unsigned long g_ulTIMERPeriph[3] =
{
#if defined(PART_TM4C1233H6PM) || defined(PART_LM4F120H5QR)
//  SYSCTL_PERIPH_TIMER0, // wiring_analog.c analogWrite()
  SYSCTL_PERIPH_TIMER1,
  SYSCTL_PERIPH_TIMER2,
  SYSCTL_PERIPH_TIMER3
//  SYSCTL_PERIPH_TIMER4, // Tone.c
//  SYSCTL_PERIPH_TIMER5, // wiring.c - millis() micros()
//  SYSCTL_PERIPH_TIMER6,
//  SYSCTL_PERIPH_TIMER7
#else
  #error "**** No PART defined or unsupported PART ****"
#endif
};

static const unsigned long g_ulTIMERBase[3] =
{
#if defined(PART_TM4C1233H6PM) || defined(PART_LM4F120H5QR)
//  TIMER0_BASE, // wiring_analog.c analogWrite()
  TIMER1_BASE,
  TIMER2_BASE,
  TIMER3_BASE
//  TIMER4_BASE, // Tone.c
//  TIMER5_BASE, // wiring.c - millis() micros()
//  TIMER6_BASE,
//  TIMER7_BASE
#else
  #error "**** No PART defined or unsupported PART ****"
#endif
};

class iAccelStepper : public AccelStepper
{
public:
  iAccelStepper() : AccelStepper() {};
  void begin(uint8_t pin1 = 2, uint8_t pin2 = 3);
  void moveTo(long absolute);
  void move(long relative);
  boolean run(void) { return running; };
  void ISR(void);
private:
  boolean running;
  unsigned int id;
};

#endif
