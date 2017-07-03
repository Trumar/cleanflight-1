#pragma once

#include "config/parameter_group.h"
#include "common/time.h"
#include "drivers/sonar_hcsr04.h"
#include "sensors/battery.h"

void calculateEstimatedWall(timeUs_t currentTimeUs);
void wallFollow(void);
