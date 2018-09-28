/*
 * Stepper motor driver for DRV8885
 *
 * Created: 07/08/2018
 *  Author: john.whittington
 */

#ifndef DRV8885_H
#define DRV8885_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEFAULT_SPS               1000
#define DEFAULT_PRESCALER         TCC_CTRLA_PRESCALER_DIV1
#define TCC0_PERIOD_COUNT         (uint16_t) (F_CPU / DEFAULT_SPS) // run off main clock with no divide

extern const uint8_t drv_enable_port, drv_enable_port_pin;
extern const uint8_t drv_step_port, drv_step_port_pin, drv_step_pin, drv_step_mux_pin;

enum class MicroStepMode {
  FullStep = 1,
  HalfStep = 2,
  QuarterStep = 4,
  EigthStep = 8,
  SixteenStep = 16
};

enum class TrqMode {
  HundredPercent,
  SeventyFivePercent,
  FiftyPercent
};

class DRV8885 {
  public:
    DRV8885(int8_t direction_pin, int8_t enable_pin, int8_t sleep_pin=-1, int8_t m0_pin=-1, int8_t m1_pin=-1, int8_t trq_pin=-1, int8_t fault_pin=-1);
    void begin(uint16_t sps = DEFAULT_SPS, int16_t prescaler = -1);
    void setStepsSecond(uint16_t steps, uint16_t prescaler);
    inline void _enablePinWO(void);
    inline void _disablePinWO(void);
    void _enableOvfInt(void);
    void _disableOvfInt(void);
    void move(bool direction, uint16_t steps);
    void move(bool direction);
    void step(bool direction);
    void stop(void);
    void enable(void);
    void disable(void);
    void sleep(void);
    bool isMoving(bool *dir);
    bool isMoving(void);
    void setMicrostepMode(MicroStepMode mode);
    void setTrqMode(TrqMode mode);
    bool getFault(void);
    int8_t _enable_pin;
    int16_t _prescaler;
    uint16_t _top_value;
    bool _moving, manual;
    volatile uint32_t _count;
    volatile uint32_t _stop_count;

  private:
    void tcc0Setup(void);
    void stepPinSetup(void);
    uint32_t readTCCCount(void);
    int8_t _direction_pin, _sleep_pin, _m0_pin, _m1_pin, _trq_pin, _fault_pin;
    bool _direction;
    MicroStepMode _stepping;
};

#endif
