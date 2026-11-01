#include "main.h"
#include "control_loop.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
    ControlLoop_Tick(HAL_GetTick());
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}
