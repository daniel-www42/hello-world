#ifndef __TASK_H
#define __TASK_H

#include "main.h"

extern volatile uint32_t tick;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim2; // 注意：如果你用的不是TIM2，请修改为对应句柄

void Task_Init(void);

#endif