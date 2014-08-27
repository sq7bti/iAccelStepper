#include "iAccelStepper.h"
#include "driverlib/timer.h"

static iAccelStepper* me[3];
static unsigned int all_instances;
static unsigned long ulPeriod;

void iAccelStepper::ISR(void) {
  TimerIntClear(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);

  if(_direction == DIRECTION_CW)
    // Clockwise
    ++_currentPos;
  else
    // Anticlockwise
    --_currentPos;

  // prepare for the next period
  computeNewSpeed();
  digitalWrite(_pin[1], _direction);

  // switch off timer at the falling edge
  if((_stepInterval == 0) || (abs(distanceToGo()) < 1)) {
    TimerDisable(g_ulTIMERBase[id], TIMER_A);
    running = false;
  } else {
    digitalWrite(_pin[0], true);
    delayMicroseconds(ulPeriod);
    digitalWrite(_pin[0], false);

    TimerLoadSet(g_ulTIMERBase[id], TIMER_A, _stepInterval - ulPeriod);
    TimerEnable(g_ulTIMERBase[id], TIMER_A);
  }
}

void timerISR0(void) { me[0]->ISR(); }
void timerISR1(void) { me[1]->ISR(); }
//void timerISR2(void) { me[2]->ISR(); }
//void timerISR3(void) { me[3]->ISR(); }
//void timerISR4(void) { me[4]->ISR(); }

typedef void (*ISR_ptr_t)(void);
//ISR_ptr_t timerISR_ptr[5] = { &timerISR0, &timerISR1, timerISR2, timerISR3, timerISR4 };
ISR_ptr_t timerISR_ptr[2] = { timerISR0, timerISR1 };

void iAccelStepper::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3)
{
  //                                        STEP  DIR
  AccelStepper::begin(AccelStepper::DRIVER, pin1, pin2);
  //                         ENABLE
  AccelStepper::setEnablePin(pin3);
  AccelStepper::setPinsInverted(false, false, false, false, true);

//  ulPeriod = 2 * clockCyclesPerMicrosecond();
  ulPeriod = 1;

  if(all_instances < 2) {
    id = all_instances;
    // Configure timer
    SysCtlPeripheralEnable(g_ulTIMERPeriph[id]);

    TimerConfigure(g_ulTIMERBase[id], TIMER_CFG_ONE_SHOT);
    TimerIntEnable(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);
    TimerIntRegister(g_ulTIMERBase[id], TIMER_A, timerISR_ptr[id]);

    me[id] = this;
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
    running = true;
    // enable driver
//    enableOutputs();
    computeNewSpeed();
    digitalWrite(_pin[1], _direction);

    if(_direction == DIRECTION_CW)
      // Clockwise
      ++_currentPos;
    else
      // Anticlockwise
      --_currentPos;

    digitalWrite(_pin[0], true);
    delayMicroseconds(ulPeriod);
    digitalWrite(_pin[0], false);

    TimerLoadSet(g_ulTIMERBase[id], TIMER_A, _stepInterval - ulPeriod);
    TimerEnable(g_ulTIMERBase[id], TIMER_A);
  }
}
