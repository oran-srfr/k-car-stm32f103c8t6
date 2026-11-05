#include "main.h"
#include "app_modules.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    __disable_irq();
    Modules_EmergencyStop();
    while (1) {
    }
}

void MemManage_Handler(void)
{
    __disable_irq();
    Modules_EmergencyStop();
    while (1) {
    }
}

void BusFault_Handler(void)
{
    __disable_irq();
    Modules_EmergencyStop();
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    __disable_irq();
    Modules_EmergencyStop();
    while (1) {
    }
}
