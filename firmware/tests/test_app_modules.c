#include "app.h"
#include "app_modules.h"
#include "Attitude.h"
#include "HCI.h"
#include "buzzer.h"
#include "lcd.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "ultrasonic.h"
#include "vehicle_control.h"
#include "control_loop.h"
#include "encoder.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick, last_motor_tick, max_motor_gap;
static bool motor_sample_seen;
static unsigned moving_glyphs;
static uint8_t raw_bits = 0x0AU;
static char screen[17];
volatile uint8_t g_motor_driver_enabled;
volatile int16_t g_motor_driver_left_actual, g_motor_driver_right_actual;
uint32_t HCI_Lap1Ms, HCI_Lap2Ms;
uint16_t HCI_ObsCount, HCI_RevCount, HCI_SideCount;

uint32_t HAL_GetTick(void) { return tick; }
void Encoder_Init(void) { }
EncoderCounts Encoder_Read(void) { return (EncoderCounts){0}; }
static void advance(unsigned ms)
{
    while (ms--) { ++tick; ControlLoop_Tick(tick); }
}
void HCI_Init(void) { }
void HCI_Update(void) { }
void LCD_Init(void) { memset(screen, ' ', 16U); screen[16] = '\0'; }

/* Model a 2 ms/glyph software SPI cost. The application, module adapter,
 * follower, PD sampling and vehicle output gate all execute real code. */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t c, uint16_t fg,
                  uint16_t bg, uint8_t size, uint8_t mode)
{
    (void)fg; (void)bg; (void)size; (void)mode;
    assert(y == 112U && x % 8U == 0U && x < 128U);
    screen[x / 8U] = (char)c;
    if (g_motor_output_enabled) ++moving_glyphs;
    advance(2U);
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *s,
                    uint16_t fg, uint16_t bg, uint8_t size, uint8_t mode)
{
    while (*s) {
        LCD_ShowChar(x, y, *s++, fg, bg, size, mode);
        x += 8U;
    }
}

void LineSensor_Init(void) { }
LineSensorRaw_t LineSensor_ReadRaw(void)
{
    return (LineSensorRaw_t){raw_bits & 1U, (raw_bits >> 1U) & 1U,
                            (raw_bits >> 2U) & 1U, (raw_bits >> 3U) & 1U};
}
void MotorDriver_Stop(void) { g_motor_driver_left_actual = g_motor_driver_right_actual = 0; }
void MotorDriver_Disable(void) { g_motor_driver_enabled = 0U; MotorDriver_Stop(); }
void MotorDriver_Init(void) { MotorDriver_Disable(); }
void MotorDriver_Enable(void) { g_motor_driver_enabled = 1U; }
void MotorDriver_SetBoth(int16_t left, int16_t right)
{
    if (motor_sample_seen && tick - last_motor_tick > max_motor_gap) {
        max_motor_gap = tick - last_motor_tick;
    }
    last_motor_tick = tick;
    motor_sample_seen = true;
    g_motor_driver_left_actual = left;
    g_motor_driver_right_actual = right;
}

bool Buzzer_Init(void) { return true; }
void Buzzer_Task(uint32_t now) { (void)now; }
bool Buzzer_Play(BuzzerEvent e, uint32_t now) { (void)e; (void)now; return true; }
bool Ulr_Init(uint32_t now) { (void)now; return true; }
void Ulr_Task(uint32_t now) { (void)now; }
UlrReading Ulr_GetReading(void) { return (UlrReading){.status = ULR_OK, .distance_mm = 1000U, .sample_ms = tick}; }
bool Attitude_Init(uint32_t baud) { (void)baud; return true; }
void Attitude_Poll(uint32_t now) { (void)now; }
void Attitude_StartLap(uint32_t now) { (void)now; }
void Attitude_StopLap(void) { }
void Attitude_GetStatus(AttitudeStatus *s) { *s = (AttitudeStatus){0}; }

int main(void)
{
    App_Init(tick);
    while (tick < 250U) {
        App_Task(tick);
        advance(1U);
    }
    assert(App_GetStatus().state == APP_FOLLOW_LINE && g_motor_output_enabled);
    assert(strstr(screen, FIRMWARE_REVISION_TEXT) != NULL);
    for (unsigned cycle = 0U; cycle < 2000U; ++cycle) {
        /* Alternate tracking and a real corner so debug T/P fields change. */
        raw_bits = cycle % 400U < 200U ? 0x08U : 0x0AU;
        App_Task(tick);
        assert(g_motor_output_enabled && App_GetStatus().state == APP_FOLLOW_LINE);
        advance(1U);
    }
    assert(moving_glyphs > 0U && max_motor_gap <= 1U);
    /* A whole foreground operation can block while timer control continues. */
    uint32_t before = g_control_call_count;
    raw_bits = 0x0AU;
    advance(80U);
    assert(g_control_call_count - before == 80U && max_motor_gap == 1U);
    before = g_control_call_count;
    ControlLoop_Tick(tick); /* duplicate dispatch at one timestamp is ignored */
    assert(g_control_call_count == before);
    g_app_control_request = APP_CONTROL_STOP;
    App_Task(tick);
    assert(!g_motor_output_enabled && !g_motor_driver_enabled);
    advance(100U);
    App_Task(tick);
    assert(screen[1] == '0' && screen[7] == '0');
    printf("PASS: module integration, auto start, bounded debug display (max motor gap %u ms), stop refresh\n",
           (unsigned)max_motor_gap);
    return 0;
}
