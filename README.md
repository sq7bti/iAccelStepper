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

Within Energia on LM4F target there are only two timers available for free use: Timer1 and Timer2

- //  SYSCTL_PERIPH_TIMER0, // wiring_analog.c analogWrite()
- //  SYSCTL_PERIPH_TIMER1,
-   SYSCTL_PERIPH_TIMER2,
-   SYSCTL_PERIPH_TIMER3
- //  SYSCTL_PERIPH_TIMER4, // Tone.c
- //  SYSCTL_PERIPH_TIMER5, // wiring.c - millis() micros()
- //  SYSCTL_PERIPH_TIMER6,
- //  SYSCTL_PERIPH_TIMER7
