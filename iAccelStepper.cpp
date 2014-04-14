#include "iAccelStepper.h"

#include "driverlib/timer.h"

volatile boolean state[3];
static iAccelStepper* me[3];
static unsigned int all_instances;

void iAccelStepper::ISR(void) {
  TimerIntClear(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);
  if(state[id]) {
    if(_direction == DIRECTION_CW)
      // Clockwise
      ++_currentPos;
    else
      // Anticlockwise
      --_currentPos;
    // prepare for the next period
    computeNewSpeed();
    digitalWrite(_pin[1], _direction);
    TimerLoadSet(g_ulTIMERBase[id], TIMER_A, 1);
    TimerEnable(g_ulTIMERBase[id], TIMER_A);
  } else {
    // switch off timer at the falling edge
    if(_stepInterval == 0) {
      TimerDisable(g_ulTIMERBase[id], TIMER_A);
      digitalWrite(GREEN_LED, 0);
      running = false;
    } else {
      TimerLoadSet(g_ulTIMERBase[id], TIMER_A, (80*_stepInterval)-1);
      TimerEnable(g_ulTIMERBase[id], TIMER_A);
    }
  }
  digitalWrite(_pin[0], state[id]);
  state[id] ^= 1;
}

void timerISR0(void) { me[0]->ISR(); }
void timerISR1(void) { me[1]->ISR(); }
void timerISR2(void) { me[2]->ISR(); }
//void timerISR3(void) { me[3]->ISR(); }
//void timerISR4(void) { me[4]->ISR(); }

typedef void (*ISR_ptr_t)(void);
//ISR_ptr_t timerISR_ptr[5] = { &timerISR0, &timerISR1, timerISR2, timerISR3, timerISR4 };
ISR_ptr_t timerISR_ptr[3] = { timerISR0, timerISR1, timerISR2 };

void iAccelStepper::begin(uint8_t pin1, uint8_t pin2)
{
  //                                        STEP  DIR
  AccelStepper::begin(AccelStepper::DRIVER, pin1, pin2);

  if(all_instances < 3) {
    id = all_instances;
    Serial.println("configure timer");
    // Configure timer
    SysCtlPeripheralEnable(g_ulTIMERPeriph[id]);
    Serial.println("configure sysctl");

    TimerConfigure(g_ulTIMERBase[id], TIMER_CFG_ONE_SHOT);
    Serial.println("configure timerconfigure");

    TimerIntEnable(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);
    Serial.println("configure timerintenable");

    TimerIntRegister(g_ulTIMERBase[id], TIMER_A, timerISR_ptr[id]);

    me[id] = this;
    state[id] = false;
    running = false;
    ++all_instances;
  }
}

void iAccelStepper::move(long relative)
{
  moveTo(_currentPos + relative);
}

void iAccelStepper::moveTo(long absolute)
{
  AccelStepper::moveTo(absolute);

  if(!running && (distanceToGo() != 0)) {
    digitalWrite(_pin[1], _direction);
    computeNewSpeed();
    state[id] = true;
    digitalWrite(_pin[0], state[id]);
    running = true;
    TimerLoadSet(g_ulTIMERBase[id], TIMER_A, 1);
    TimerEnable(g_ulTIMERBase[id], TIMER_A);
    digitalWrite(GREEN_LED, 1);
  }
}
