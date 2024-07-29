/*  Portions of this file are an adaptation of the SimpleISA library, originally authored by Jack Rickard.
 *
 *  At present, this code supports the Scale IVT Modular current/voltage sensor device.  
 *  These devices measure current, up to three voltages, and provide temperature compensation.
 *  Additional sensors are planned to provide flexibility/lower BOM costs.
 *
 *  Original license/copyright header of SimpleISA is shown below:
 *   This library was written by Jack Rickard of EVtv - http://www.evtv.me
 *   copyright 2014
 *   You are licensed to use this library for any purpose, commercial or private, 
 *   without restriction.
 *
 *  2024 - Modified to make use of ESP32-Arduino-CAN by miwagner
 *
 */
#include "../include.h"
#ifdef CHADEMO_BATTERY
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/CAN_config.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/ESP32CAN.h"
#include "CHADEMO-BATTERY-INTERNAL.h"
#include "CHADEMO-BATTERY.h"
#include "CHADEMO-SHUNTS.h"

/* Initial frames received from ISA shunts provide invalid data during initialization */
static int framecount = 0;

static float Amperes = 0;  // Floating point with current in Amperes
static double AH = 0.0;    //Floating point with accumulated ampere-hours
static double KW = 0.0;
static double KWH = 0.0;

static double Voltage = 0.0;
static double Voltage1 = 0.0;
static double Voltage2 = 0.0;
static double Voltage3 = 0.0;

static double Temperature = 0.0;

static double milliamps = 0.0;
static long As = 0;
static long lastAs = 0;
static long watt = 0;
static long wh = 0;
static long lastWh = 0;

/* Default cmd CAN frame ID for ISA IVT-S, though it is configurable.
 *  If it differs from expectations, a warning will be emitted.
 *
 */
#define ISA_CMD_CANID 0x411

/* Frame for triggering ISA IVT-S commands such as mode changes, configuration updates, resets, etc */
CAN_frame_t ISA_outframe = {.FIR = {.B =
                                        {
                                            .DLC = 8,
                                            .unknown_2 = 0,
                                            .RTR = CAN_no_RTR,
                                            .FF = CAN_frame_std,
                                        }},

                            .MsgID = ISA_CMD_CANID,
                            .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

double ISA_get_measured_voltage1() {
  return Voltage1;
}
double ISA_get_measured_voltage2() {
  return Voltage2;
}
double ISA_get_measured_voltage3() {
  return Voltage3;
}

float ISA_get_measured_current() {
  return Amperes;
}

//This is our CAN interrupt service routine to catch inbound frames
void ISA_handleFrame(CAN_frame_t* frame) {

  if (!frame)
    return;

  framecount++;

  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;  //We are getting CAN messages

  switch (frame->MsgID) {
    case 0x511:
      if (frame->data.u8[0] == 0xBF) {
        Serial.print("ISA IVT ready");
        uint16_t cmdid = frame->data.u8[2] << 8 | frame->data.u8[1];
        if (cmdid != ISA_CMD_CANID) {
          Serial.println("Unknown/incompatible IVT configuration or model?");
        }
      }

      break;

    case 0x521:
      ISA_handleAmperage_521(frame);
      break;

    case 0x522:
      ISA_handleVoltage1_522(frame);
      break;

    case 0x523:
      ISA_handleVoltage2_523(frame);
      break;

    case 0x524:
      ISA_handleVoltage3_524(frame);
      break;

    case 0x525:
      ISA_handleTemperature_525(frame);
      break;

    case 0x526:
      ISA_handleWatts_526(frame);
      break;

    case 0x527:
      ISA_handleAmpHours_527(frame);
      break;

    case 0x528:
      ISA_handleWattHours_528(frame);
      break;

    default:
      Serial.print("Unhandled frame ID=");
      Serial.println(frame->MsgID, HEX);
      break;
  }

  return;
}

//handle frame for Amperes
inline void ISA_handleAmperage_521(CAN_frame_t* frame) {

  if (!frame)
    return;

  milliamps =
      (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));

  Amperes = milliamps / 1000.0f;
}

//handle frame for Voltage
inline void ISA_handleVoltage1_522(CAN_frame_t* frame) {

  if (!frame)
    return;

  long volt =
      (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));

  Voltage = volt / 1000.0f;
  Serial.print("Voltage1: ");
  Serial.println(Voltage);

  Voltage1 = Voltage - (Voltage2 + Voltage3);
}

//handle frame for Voltage 2
inline void ISA_handleVoltage2_523(CAN_frame_t* frame) {

  if (!frame)
    return;

  long volt =
      (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));

  Voltage2 = volt / 1000.0f;

  Serial.print("Voltage2: ");
  Serial.println(Voltage2);

  if (Voltage2 > 3)
    Voltage2 -= Voltage3;
}

//handle frame for Voltage3
inline void ISA_handleVoltage3_524(CAN_frame_t* frame) {

  if (!frame)
    return;

  long volt =
      (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));

  Voltage3 = volt / 1000.0f;
}

//handle frame for Temperature (C) reported in whole degree increment, but granular to 0.1 °C in packet
inline void ISA_handleTemperature_525(CAN_frame_t* frame) {

  if (!frame)
    return;

  long temp =
      (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));

  Temperature = temp / 10;
}

//handle frame for Kilowatts
inline void ISA_handleWatts_526(CAN_frame_t* frame) {

  if (!frame)
    return;

  watt = (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));
  KW = watt / 1000.0f;
}

//handle frame for Ampere-Hours
inline void ISA_handleAmpHours_527(CAN_frame_t* frame) {

  if (!frame)
    return;

  As = (frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]);

  AH += (As - lastAs) / 3600.0f;
  lastAs = As;
}

//handle frame for kiloWatt-hours
inline void ISA_handleWattHours_528(CAN_frame_t* frame) {

  if (!frame)
    return;

  wh = (long)((frame->data.u8[5] << 24) | (frame->data.u8[4] << 16) | (frame->data.u8[3] << 8) | (frame->data.u8[2]));
  KWH += (wh - lastWh) / 1000.0f;
  lastWh = wh;
}

/* current_mode @1 = start, @0 = stop
 * restart_mode @1 = start, @0 = stop
 */
void ISA_set_mode(bool current_mode, bool restart_mode) {
  ISA_outframe.data.u8[0] = 0x34; /* mode set subcommand */
  ISA_outframe.data.u8[1] = current_mode;

  ISA_outframe.data.u8[2] = restart_mode;

  ISA_outframe.data.u8[3] = 0x00;
  ISA_outframe.data.u8[4] = 0x00;
  ISA_outframe.data.u8[5] = 0x00;
  ISA_outframe.data.u8[6] = 0x00;
  ISA_outframe.data.u8[7] = 0x00;
  ESP32Can.CANWriteFrame(&ISA_outframe);
}

#endif
