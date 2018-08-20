/* Project: DRV8885 Demo */

#include <Arduino.h>
#include <Wire.h>

#include <DRV8885.h>

#define STEP_PIN             17 // PA04
#define STEP_PORT            PORTA
#define STEP_PORT_PIN        PORT_PA04
#define STEP_UC_PIN          PIN_PA04
#define STEP_MUX_PIN         PORT_PMUX_PMUXE_E
#define DIRECTION_PIN        18 // PA05
#define ENABLE_PIN           8  // PA06
#define SLEEP_PIN            9  // PA07
#define M0_PIN               38 // PA13
#define M1_PIN               11 // PA16
#define TRQ_PIN              2  // PA14
#define FAULT_PIN            27 // PA28

// DRV8885 port/pin config
const uint8_t drv_step_port = STEP_PORT, drv_step_port_pin = STEP_PORT_PIN, drv_step_pin = STEP_UC_PIN, drv_step_mux_pin = STEP_MUX_PIN;
// Linear actuator is stepper motor driven by DRV8885
DRV8885 linearActuator(DIRECTION_PIN, ENABLE_PIN, SLEEP_PIN, M0_PIN, M1_PIN, TRQ_PIN, FAULT_PIN);

void setup() {
  linearActuator.begin();
}

void loop() {
  static bool toggle = 0;

  linearActuator.move(toggle, 1000); // move 1000 steps then stop
  /* linearActuator.move(toggle); */ // uncomment to move for 500 ms back and fourth
  toggle != toggle;
  delay(500);
}
