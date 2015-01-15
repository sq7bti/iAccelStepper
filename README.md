iAccelStepper
=============

interrupt driven stepper controller inherited from AccelStepper

To use this module inherit from the AccelStepper class (http://www.airspayce.com/mikem/arduino/AccelStepper/), modify it to expose necessary members into public:

* _stepInterval
* _direction
* _currentPos
* _targetPos
* _speed
* _pin[4]

Easiest is to use provided patch https://github.com/sq7bti/iAccelStepper/blob/master/AccelStepper.patch

Within Energia on LM4F target there are only two timers available for free use: Timer1 and Timer2

- //  SYSCTL_PERIPH_TIMER0, // wiring_analog.c analogWrite()
-   SYSCTL_PERIPH_TIMER1,
-   SYSCTL_PERIPH_TIMER2,
-   SYSCTL_PERIPH_TIMER3
- //  SYSCTL_PERIPH_TIMER4, // Tone.c
- //  SYSCTL_PERIPH_TIMER5, // wiring.c - millis() micros()
- //  SYSCTL_PERIPH_TIMER6,
- //  SYSCTL_PERIPH_TIMER7

To sum up, both the original AccelStepper and iAccelStepper library should be located in your library folder, not neceserily in a common folder:
${ENERGIA_PROJECT}/sketches
${ENERGIA_PROJECT}/libraries/AccelStepper/AccelStepper.cpp
${ENERGIA_PROJECT}/libraries/AccelStepper/AccelStepper.h
${ENERGIA_PROJECT}/libraries/iAccelStepper/iAccelStepper.cpp
${ENERGIA_PROJECT}/libraries/iAccelStepper/iAccelStepper.h

In the main ino file one need to include both header files:
```
#include "AccelStepper.h"
#include "iAccelStepper.h"
```

Instantiation of object boils down to an empty declaration:
```
iAccelStepper axis1;
iAccelStepper axis2;
```
setup() should contain initialisation call:
```
axis1.begin(PB_0, PB_5);
axis2.begin(PE_4, PB_1);
```
To perform any movement, the usual methods of the original AccelStepper should be used, such as move(), moveTo() and stop(). The difference is iAccelStepper _DO_NOT_REQUIRE_ call run() method. See example for more.

