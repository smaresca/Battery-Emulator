#ifndef CHADEMO_BATTERY_H
#define CHADEMO_BATTERY_H
#include <Arduino.h>
#include "../include.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/ESP32CAN.h"

#define BATTERY_SELECTED
#define MAX_CELL_DEVIATION_MV 9999

/* CHADEMO handling runs at 6.25 times the rate of most other code, so, rather than the
 *  default value of 12 (for 12 iterations of the 5s value update loop) * 5 for a 60s timeout,
 *  instead use 75 for 75*0.8s = 60s
 */
#undef CAN_STILL_ALIVE
#define CAN_STILL_ALIVE 75

//Contactor control is required for CHADEMO support
#define CONTACTOR_CONTROL

//ISA shunt is currently required for CHADEMO support
// other measurement sources may be added in the future
#define ISA_SHUNT

void setup_battery(void);

#endif
