#ifndef TEACH_DRIVE_H
#define TEACH_DRIVE_H

#include "stm32f1xx_hal.h"

_Noreturn void TeachDrive_Run(I2C_HandleTypeDef *bus);

#endif
