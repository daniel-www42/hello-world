#include "task.h"

volatile uint32_t tick = 0;

void Task_Init(void)
{
    // 第1题：F103最小系统板 PC13 低电平点亮（其他板子改对应引脚）
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    
    // 开启定时器中断
    HAL_TIM_Base_Start_IT(&htim2);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) 
    {
        tick++; // 每1ms加1
        
        //第2题（定时器题）：保留这行代码
        //HAL_IWDG_Refresh(&hiwdg);
        
        // 第3题（看门狗题）：
        //HAL_IWDG_Refresh(&hiwdg);
    }
}