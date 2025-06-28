#ifndef __PATH_H
#define __PATH_H

#include "stdint.h"
#include "DJI_motor.h"

void path_all_cmd_Task();
void path_init();
void xianwei_check(DJIMotor_Instance *motor);

#endif
