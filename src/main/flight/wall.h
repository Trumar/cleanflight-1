#pragma once

#include "config/parameter_group.h"
#include "common/time.h"
#include "drivers/sonar_hcsr04.h"
#include "sensors/battery.h"

int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old);
void calculateEstimatedWall(timeUs_t currentTimeUs);
void wallFollow(void);
