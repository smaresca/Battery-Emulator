#ifndef BATTERIES_H
#define BATTERIES_H

#ifdef BATTERY_TYPE_LEAF
  #include "NISSAN-LEAF-BATTERY.h" //See this file for more LEAF battery settings
#endif

#ifdef TESLA_MODEL_3_BATTERY
  #include "TESLA-MODEL-3-BATTERY.h" //See this file for more Tesla battery settings
#endif

#ifdef RENAULT_ZOE_BATTERY
  #include "RENAULT-ZOE-BATTERY.h" //See this file for more Zoe battery settings
#endif

#ifdef BMW_I3_BATTERY
  #include "BMW-I3-BATTERY.h" //See this file for more i3 battery settings
#endif

#ifdef IMIEV_ION_CZERO_BATTERY
  #include "IMIEV-CZERO-ION-BATTERY.h" //See this file for more triplet battery settings
#endif

#ifdef KIA_HYUNDAI_64_BATTERY
  #include "KIA-HYUNDAI-64-BATTERY.h" //See this file for more 64kWh battery settings
#endif

#ifdef LUNA2000
  #include "LUNA2000-BATTERY.h"
#endif

#ifdef CHADEMO
	#include "CHADEMO-BATTERY.h" //See this file for more Chademo settings
#endif

#endif
