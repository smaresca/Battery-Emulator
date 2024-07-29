#ifndef CHADEMO_SHUNTS_H
#define CHADEMO_SHUNTS_H

double ISA_get_measured_voltage1();
double ISA_get_measured_voltage2();
double ISA_get_measured_voltage3();
float ISA_get_measured_current();
void ISA_handleFrame(CAN_frame_t* frame);
inline void ISA_handleAmperage_521(CAN_frame_t* frame);
inline void ISA_handleVoltage1_522(CAN_frame_t* frame);
inline void ISA_handleVoltage2_523(CAN_frame_t* frame);
inline void ISA_handleVoltage3_524(CAN_frame_t* frame);
inline void ISA_handleTemperature_525(CAN_frame_t* frame);
inline void ISA_handleWatts_526(CAN_frame_t* frame);
inline void ISA_handleAmpHours_527(CAN_frame_t* frame);
inline void ISA_handleWattHours_528(CAN_frame_t* frame);
void ISA_set_mode(bool current_mode, bool restart_mode);
#endif
