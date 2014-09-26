#include "iAccelStepper.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"

//*****************************************************************************
//
// The list of Timer peripherals.
//
//*****************************************************************************
static const unsigned long g_ulTIMERPeriph[MAX_INST] =
{
#if defined(PART_TM4C1233H6PM) || defined(PART_LM4F120H5QR)
//  SYSCTL_PERIPH_TIMER0, // wiring_analog.c analogWrite()
//  SYSCTL_PERIPH_TIMER1,
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

static const unsigned long g_ulTIMERBase[MAX_INST] =
{
#if defined(PART_TM4C1233H6PM) || defined(PART_LM4F120H5QR)
//  TIMER0_BASE, // wiring_analog.c analogWrite()
//  TIMER1_BASE,
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

static iAccelStepper* me[MAX_INST];
static uint32_t _port_step[MAX_INST];
static uint8_t _pin_step[MAX_INST];
static uint32_t _port_dir[MAX_INST];
static uint8_t _pin_dir[MAX_INST];
static boolean dir[MAX_INST];
static boolean _state[MAX_INST];
static boolean _setspeed[MAX_INST];
static unsigned char all_instances;
static uint32_t ulPeriod;

void iAccelStepper::ISR(void) {
  //TimerIntClear(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);
  HWREG(g_ulTIMERBase[id] + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

  // falling edge produce space for length _stepInterval
  if(_state[id]) {
    _state[id] = false;

    // prepare for the next period
    // falling edge - calculate everything necessary and calculate _stepInterval
    if(!_setspeed[id])
      computeNewSpeed();

    if(dir[id] != _direction) {
      dir[id] = _direction;
      HWREG(_port_dir[id]) = _direction?_pin_dir[id]:0;
    }

    // either fire the timer again for another period or switch it off when the move is finished
//    if((_stepInterval == 0) || (abs(distanceToGo()) < 1)) {
    if((_stepInterval == 0) && (!_setspeed[id])) {
      //TimerDisable(g_ulTIMERBase[id], TIMER_A);
      HWREG(g_ulTIMERBase[id] + TIMER_O_CTL) &= ~(TIMER_A & (TIMER_CTL_TAEN | TIMER_CTL_TBEN));
      running = false;
    } else {
      HWREG(_port_step[id]) = 0;
      //TimerLoadSet(g_ulTIMERBase[id], TIMER_A, _stepInterval - ulPeriod);
      HWREG(g_ulTIMERBase[id] + TIMER_O_TAILR) = _stepInterval - ulPeriod;
      //TimerEnable(g_ulTIMERBase[id], TIMER_A);
      HWREG(g_ulTIMERBase[id] + TIMER_O_CTL) |= TIMER_A & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
    }
  } else {

    _state[id] = true;

    if(_direction == DIRECTION_CW)
      // Clockwise
      ++_currentPos;
    else
      // Anticlockwise
      --_currentPos;

    //TimerLoadSet(g_ulTIMERBase[id], TIMER_A, ulPeriod);
    HWREG(g_ulTIMERBase[id] + TIMER_O_TAILR) = ulPeriod;
    HWREG(_port_step[id]) = _pin_step[id];
    //TimerEnable(g_ulTIMERBase[id], TIMER_A);
    HWREG(g_ulTIMERBase[id] + TIMER_O_CTL) |= TIMER_A & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
  }
}

void timerISR0(void) { me[0]->ISR(); }
void timerISR1(void) { me[1]->ISR(); }
//void timerISR2(void) { me[2]->ISR(); }
//void timerISR3(void) { me[3]->ISR(); }
//void timerISR4(void) { me[4]->ISR(); }

typedef void (*ISR_ptr_t)(void);
//ISR_ptr_t timerISR_ptr[5] = { &timerISR0, &timerISR1, timerISR2, timerISR3, timerISR4 };
ISR_ptr_t timerISR_ptr[MAX_INST] = { timerISR0, timerISR1 };

void iAccelStepper::begin(uint8_t pin1, uint8_t pin2, uint8_t pin3)
{
  //                                        STEP  DIR
  AccelStepper::begin(AccelStepper::DRIVER, pin1, pin2);
  //                         ENABLE
  AccelStepper::setEnablePin(pin3);
  AccelStepper::setPinsInverted(false, false, true);

  // specs of DRV8825 requires 2us, A3967 and A4988 requires atleast 1us step pulse
//  ulPeriod = clockCyclesPerMicrosecond() * 2;
  ulPeriod = 1;

  if(all_instances < MAX_INST) {
    id = all_instances;
    // Configure timer
    SysCtlPeripheralEnable(g_ulTIMERPeriph[id]);

    TimerConfigure(g_ulTIMERBase[id], TIMER_CFG_ONE_SHOT);
    //TimerIntEnable(g_ulTIMERBase[id], TIMER_TIMA_TIMEOUT);
    HWREG(g_ulTIMERBase[id] + TIMER_O_IMR) |= TIMER_TIMA_TIMEOUT;
    TimerIntRegister(g_ulTIMERBase[id], TIMER_A, timerISR_ptr[id]);

    me[id] = this;
    running = false;
    ++all_instances;
    _port_step[id] = (uint32_t)portBASERegister(digitalPinToPort(pin1)) + (GPIO_O_DATA + (digitalPinToBitMask(pin1) << 2));
    _pin_step[id] = (uint8_t)digitalPinToBitMask(pin1);
    _port_dir[id]  = (uint32_t)portBASERegister(digitalPinToPort(pin2)) + (GPIO_O_DATA + (digitalPinToBitMask(pin2) << 2));
    _pin_dir[id]  = (uint8_t)digitalPinToBitMask(pin2);
    dir[id] = false;
    _state[id] = false;
    _setspeed[id] = false;
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

    if(dir[id] != _direction) {
      dir[id] = _direction;
      HWREG(_port_dir[id]) = _direction?_pin_dir[id]:0;
    }

    HWREG(_port_step[id]) = 0;
    _state[id] = false;

    //TimerLoadSet(g_ulTIMERBase[id], TIMER_A, _stepInterval - ulPeriod);
    HWREG(g_ulTIMERBase[id] + TIMER_O_TAILR) = _stepInterval - ulPeriod;
    //TimerEnable(g_ulTIMERBase[id], TIMER_A);
    HWREG(g_ulTIMERBase[id] + TIMER_O_CTL) |= TIMER_A & (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
  }
}

void iAccelStepper::setSpeed(float speed)
{
  if(abs(speed) > 0.000001) {
    _setspeed[id] = true;
    AccelStepper::setSpeed(speed);
  } else {
    _setspeed[id] = false;
  }
}
