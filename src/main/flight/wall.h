#pragma once

#include "config/parameter_group.h"
#include "common/time.h"
#include "drivers/sonar_hcsr04.h"
#include "sensors/battery.h"

void updateWallFollowState(void);//used to update the desired wall follow distance when LEDDAR mode is enabled
int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old); //PID control to determine ROLL commands
void sendVeloctyToWall(int32_t velY_tmp, float accY_tmp, float accY_old);
void calculateEstimatedWall(timeUs_t currentTimeUs); //finds distance from wall, and acc velocity measurements
void wallFollow(void); //applies ROLL commands to FC

