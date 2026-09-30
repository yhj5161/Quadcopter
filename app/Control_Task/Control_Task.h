#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include "tim.h"
#include "usart.h"

extern uint32_t Timer_Bsp_t;               /* 程序运行的时间戳（s） */

void Control_Task_Housekeeping_Callback(API_TIM_Id_t id);  /* TIM3: 按键/时间戳 */
void Control_Task_USART_Callback(API_USART_Id_t id);      /* USART 中断分发 */

#endif /* __CONTROL_TASK_H */