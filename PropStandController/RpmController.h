#ifndef RPM_H
#define RPM_H

#include "Arduino.h"
#include <Arduino_PortentaBreakout.h>

#define RPM_UPDATE_INTERVAL 1000  // 1000 millisecond update rate.
#define MS_TO_M  60000.0

#define HALL_SENSOR_1_PIN SAI_FS
#define HALL_SENSOR_2_PIN SAI_D0

void initRpmController(uint8_t propNumber);
float updateRpm(uint8_t propNumber);

#endif
