#include "iAccelStepper.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"

#define portOutputRegister(x) (regtype)portBASERegister(x)
typedef volatile uint32_t regtype;
typedef uint8_t regsize;

static iAccelStepper* me[2];
static unsigned int ulPort_step[2];
static unsigned int ucPin_step[2];
static unsigned int ulPort_dir[2];
static unsigned int ucPin_dir[2];
static boolean direction[2];
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
  if(direction[id] != _direction) {
    direction[id] = _direction;
    HWREG(ulPort_dir + (GPIO_O_DATA + (ucPin_dir[id]))) = _direction;
  }

  // switch off timer at the falling edge
  if((_stepInterval == 0) || (abs(distanceToGo()) < 1)) {
    TimerDisable(g_ulTIMERBase[id], TIMER_A);
    running = false;
  } else {
    HWREG(ulPort_step + (GPIO_O_DATA + (ucPin_step[id]))) = true;
    delayMicroseconds(ulPeriod);
    HWREG(ulPort_step + (GPIO_O_DATA + (ucPin_step[id]))) = false;

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
    ulPort_step[id] = portOutputRegister(digitalPinToPort(pin1));
    ucPin_step[id] = digitalPinToBitMask(pin1);
    ulPort_dir[id] = portOutputRegister(digitalPinToPort(pin2));
    ucPin_dir[id] = digitalPinToBitMask(pin2);
    direction[id] = false;
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
    if(direction[id] != _direction) {
      direction[id] = _direction;
      HWREG(ulPort_dir + (GPIO_O_DATA + (ucPin_dir[id]))) = _direction;
    }

    if(_direction == DIRECTION_CW)
      // Clockwise
      ++_currentPos;
    else
      // Anticlockwise
      --_currentPos;

    HWREG(ulPort_step + (GPIO_O_DATA + (ucPin_step[id]))) = true;
    delayMicroseconds(ulPeriod);
    HWREG(ulPort_step + (GPIO_O_DATA + (ucPin_step[id]))) = false;

    TimerLoadSet(g_ulTIMERBase[id], TIMER_A, _stepInterval - ulPeriod);
    TimerEnable(g_ulTIMERBase[id], TIMER_A);
  }
}
