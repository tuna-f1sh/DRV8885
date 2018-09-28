/*
 * Stepper motor driver for DRV8885
 *
 * Created: 07/08/2018
 *  Author: john.whittington
 */

#include "Arduino.h"

#include "DRV8885.h"

#define DIGITAL_SET_DIRECT(x) PORT->Group[g_APinDescription[x].ulPort].OUTSET.reg = (1ul << g_APinDescription[x].ulPin)
#define DIGITAL_CLR_DIRECT(x) PORT->Group[g_APinDescription[x].ulPort].OUTCLR.reg = (1ul << g_APinDescription[x].ulPin)

/* volatile static uint32_t _count; */
/* volatile static uint32_t _stop_count; */
static DRV8885 *pStepperMotor;

// Ineterrupt handler called every time the TCC0 timer overflows (each TCC0 PWM cycle)
void TCC0_Handler() {
  // Check for overflow (OVF) interrupt
  if (TCC0->INTFLAG.bit.OVF && TCC0->INTENSET.bit.OVF) {
    ++(pStepperMotor->_count);
    // if the new count is greater than stop count, turn off waveform (stepping) output
    if ((pStepperMotor->_count > pStepperMotor->_stop_count) && !pStepperMotor->manual) {  
      pStepperMotor->_disablePinWO();
      pStepperMotor->_disableOvfInt(); // stop OVF to save processor
      pStepperMotor->_moving = false;
      pStepperMotor->_count = pStepperMotor->_stop_count;
    // otherwise, if first OVF, enable waveform output (inverted means that the procedding cycle the IC will see edge - 1st step)
    } else if (!PORT->Group[drv_step_port].PINCFG[drv_step_pin].reg && !pStepperMotor->manual){
      pStepperMotor->_enablePinWO();
    // if pin is cleared and we are in manual, save the count value
    } 

    REG_TCC0_INTFLAG |= TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
  }
}

DRV8885::DRV8885(int8_t direction_pin, int8_t enable_pin, int8_t sleep_pin, int8_t m0_pin, int8_t m1_pin, int8_t trq_pin, int8_t fault_pin) {
  _enable_pin = enable_pin;
  _direction_pin = direction_pin;
  _sleep_pin = sleep_pin;
  _m0_pin = m0_pin;
  _m1_pin = m1_pin;
  _trq_pin = trq_pin;
  _fault_pin = fault_pin;
  pStepperMotor = this;
  _stepping = MicroStepMode::FullStep;
  _prescaler = DEFAULT_PRESCALER;
  _top_value = TCC0_PERIOD_COUNT;
  _direction = 0;
  _moving = false;
}

void DRV8885::tcc0Setup(void) {
  // Setup the Generic clock source to GCLK0 (48 MHz)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // .... on GCLK0...
                     GCLK_CLKCTRL_ID_EIC;         // ... to feed the GCLK0 to EIC peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // ....on GCLK0...
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // ... to feed the GCLK5 to TCC0 and TCC1 peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC0 peripheral
  // Feed GCLK0 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;         // Setup single slope PWM on TCC0
  REG_TCC0_EVCTRL |= TCC_EVCTRL_TCINV0;           // Invert
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = TCC0_PERIOD_COUNT;               // Set the frequency of the PWM on TCC0 to 1kHz
  while(TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization

  NVIC_SetPriority(TCC0_IRQn, 0);                 // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest)
  NVIC_EnableIRQ(TCC0_IRQn);                      // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)

  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE | DEFAULT_PRESCALER;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void DRV8885::setStepsSecond(uint16_t steps, uint16_t prescaler) {
  _prescaler = prescaler;
  _top_value = (uint16_t) (F_CPU / (steps * _prescaler)) - 1;
  uint16_t reg_prescaler;

  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC0 peripheral

  switch (_prescaler) {
    case 1024:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV1024;
      break;
    case 256:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV256;
      break;
    case 64:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV64;
      break;
    case 8:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV8;
      break;
    case 4:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV4;
      break;
    case 2:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV2;
      break;
    case 1:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV1;
      break;
    default:
      reg_prescaler = TCC_CTRLA_PRESCALER_DIV1;
      _prescaler = 1;
      break;
  }
  
  REG_TCC0_PER = _top_value;                      // Set the frequency of the PWM on TCC0
  while(TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization
  REG_TCC0_CCB0 = (_top_value >> 1);              // TCC0 CCB0 - on output on W0[0] (PA04) 50% duty cycle
  while(TCC0->SYNCBUSY.bit.CCB0);                 // Wait for synchronization

  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE | reg_prescaler;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
} 

void DRV8885::stepPinSetup(void) {
  REG_TCC0_CCB0 = (TCC0_PERIOD_COUNT >> 1);       // TCC0 CCB0 - on output on W0[0] (PA04) 50% duty cycle
  while(TCC0->SYNCBUSY.bit.CCB0);                 // Wait for synchronization
  // Configure pin mux for waveform output on PA04
  PORT->Group[drv_step_port].DIRCLR.reg = drv_step_port_pin;
	PORT->Group[drv_step_port].OUTCLR.reg = drv_step_port_pin;
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUXO and even PMUXE
  PORT->Group[drv_step_port].PMUX[drv_step_pin >> 1].reg = drv_step_mux_pin;
  // Enable the port multiplexer for the digital pins D7
  PORT->Group[drv_step_port].PINCFG[drv_step_pin].reg = PORT_PINCFG_PMUXEN;
}

inline void DRV8885::_enablePinWO(void) {
  PORT->Group[drv_step_port].PINCFG[drv_step_pin].reg = PORT_PINCFG_PMUXEN;
}

inline void DRV8885::_disablePinWO(void) {
  PORT->Group[drv_step_port].PINCFG[drv_step_pin].reg = 0;
}

void DRV8885::_enableOvfInt(void) {
  // only enable is not already enabled to prevent clearing valid flag
  if (!TCC0->INTENSET.bit.OVF) {
    REG_TCC0_INTFLAG |= TC_INTFLAG_OVF;               // Clear the overflow interrupt flag
    REG_TCC0_INTENSET |= TC_INTENSET_OVF;            // Enable TCC0 overflow interrupt
  }
}

void DRV8885::_disableOvfInt(void) {
  if (TCC0->INTENSET.bit.OVF) {
    REG_TCC0_INTENCLR |= TC_INTENCLR_OVF;           // Disable TCC0 overflow interrupt
    REG_TCC0_INTFLAG |= TC_INTFLAG_OVF;             // Clear the overflow interrupt flag
  }
}

uint32_t DRV8885::readTCCCount() {
  REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
  while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
  while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization  
  return REG_TCC0_COUNT;
}

void DRV8885::begin(uint16_t sps, int16_t prescaler) {
  // setup TCC0
  this->tcc0Setup();
  // enable pin WO
  this->stepPinSetup();
  if (prescaler != -1) this->setStepsSecond(sps, prescaler);
  // setup pins
  pinMode(_enable_pin, OUTPUT); pinMode(_direction_pin, OUTPUT);
  if (_sleep_pin != -1) pinMode(_sleep_pin, OUTPUT); 
  if (_m0_pin != -1) pinMode(_m0_pin, OUTPUT); 
  if (_m1_pin != -1) pinMode(_m1_pin, OUTPUT); 
  if (_trq_pin != -1) pinMode(_trq_pin, OUTPUT); 
  if (_fault_pin != -1) pinMode(_fault_pin, INPUT);
  this->setMicrostepMode(this->_stepping);

  // disable by default
  this->_disablePinWO();
  DIGITAL_CLR_DIRECT(_enable_pin);
  if (_sleep_pin != -1) DIGITAL_CLR_DIRECT(_sleep_pin);
}

void DRV8885::move(bool direction, uint16_t steps) {
  this->manual = false;
  /* steps *= (uint16_t) _stepping; */ // this happens at a device level?
  // enable the stepper motor
  if (direction)
    DIGITAL_SET_DIRECT(_direction_pin);
  else
    DIGITAL_CLR_DIRECT(_direction_pin);
  if (_sleep_pin != -1) DIGITAL_SET_DIRECT(_sleep_pin);
  DIGITAL_SET_DIRECT(_enable_pin);
  // set to current count value + desired steps - count won't have incremented as OVF but doesn't matter as long as relative
  this->_count = 0; // clear anyway
  // steps +1 with inverted waveform means that stepping output can be enabled syncronously in OVF
  this->_stop_count = this->_count + steps + 1;
  this->_enableOvfInt(); // enable OVF to disable after desired steps, while enable after first OVF, hence steps +1
  this->_direction = direction;
  this->_moving = true;
}

void DRV8885::move(bool direction) {
  this->_disableOvfInt(); // stop OVF to prevent it disabling and run indefinately
  this->manual = true;
  digitalWrite(_direction_pin, direction);
  if (direction)
    DIGITAL_SET_DIRECT(_direction_pin);
  else
    DIGITAL_CLR_DIRECT(_direction_pin);
  if (_sleep_pin != -1) DIGITAL_SET_DIRECT(_sleep_pin);
  DIGITAL_SET_DIRECT(_enable_pin);
  // enable WO since in manual mode, must be stopped using stop()
  this->_enablePinWO();
  this->_direction = direction;
  this->_moving = true;
  this->_count = 0;
}

void DRV8885::step(bool direction) {
  this->move(direction, 1);
  this->_direction = direction;
  this->_moving = true;
}

void DRV8885::stop(void) {
  this->_disablePinWO();
  this->_disableOvfInt(); // stop OVF to save processor
  this->_moving = false;
}

void DRV8885::enable(void) {
  DIGITAL_SET_DIRECT(_enable_pin);
}

void DRV8885::disable(void) {
  DIGITAL_CLR_DIRECT(_enable_pin);
}

void DRV8885::sleep(void) {
  this->stop();
  if (_sleep_pin != -1) DIGITAL_CLR_DIRECT(_sleep_pin);
}

bool DRV8885::isMoving(bool *dir) {
  *dir = this->_direction;
  return this->_moving;
}

bool DRV8885::isMoving(void) {
  return this->_moving;
}

void DRV8885::setMicrostepMode(MicroStepMode mode) {
  if ((_m0_pin != -1) && (_m1_pin != -1)) {
    _stepping = mode;
    pinMode(_m0_pin, OUTPUT);
    switch (_stepping) {
      default:
      case MicroStepMode::FullStep:
        digitalWrite(_m0_pin, LOW);
        digitalWrite(_m1_pin, LOW);
        break;
      case MicroStepMode::HalfStep:
        digitalWrite(_m0_pin, LOW);
        digitalWrite(_m1_pin, HIGH);
        break;
      case MicroStepMode::QuarterStep:
        digitalWrite(_m0_pin, HIGH);
        digitalWrite(_m1_pin, HIGH);
        break;
      case MicroStepMode::EigthStep:
        pinMode(_m0_pin, INPUT);
        digitalWrite(_m1_pin, LOW);
        break;
      case MicroStepMode::SixteenStep:
        digitalWrite(_m0_pin, HIGH);
        digitalWrite(_m1_pin, LOW);
        break;
    }
  }
}

void DRV8885::setTrqMode(TrqMode mode) {
  if (_trq_pin != -1) {
    switch (mode) {
      default:
      case TrqMode::HundredPercent:
        pinMode(_trq_pin, OUTPUT);
        digitalWrite(_trq_pin, LOW);
        break;
      case TrqMode::SeventyFivePercent:
        pinMode(_trq_pin, INPUT);
        break;
      case TrqMode::FiftyPercent:
        pinMode(_trq_pin, OUTPUT);
        digitalWrite(_trq_pin, HIGH);
        break;
    }
  }
}

bool DRV8885::getFault(void) {
  if (_fault_pin != -1) {
    return !digitalRead(_fault_pin);
  }

  return 1; // fault it pin not configured
}
