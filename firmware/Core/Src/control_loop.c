#include "control_loop.h"
#include "control_atomic.h"
#include "control_config.h"
#include "encoder.h"
#include "vehicle_control.h"
#include "wheel_speed.h"

static volatile bool s_ready;
static bool s_seen_tick;
static uint32_t s_last_tick;
static volatile struct { float yaw; uint32_t ms; bool valid; } s_yaw;
volatile uint32_t g_control_max_gap_ms;
volatile uint32_t g_control_max_cycles;
volatile uint32_t g_control_overrun_count;

void ControlLoop_Init(void)
{
  uint32_t key = ControlAtomic_Enter();
  s_ready = false;
  s_seen_tick = false;
  s_yaw.valid = false;
  g_control_max_gap_ms = 0U;
  g_control_max_cycles = g_control_overrun_count = 0U;
#if defined(__arm__) || defined(__thumb__)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
  Encoder_Init();
  s_ready = true;
  ControlAtomic_Exit(key);
}

void ControlLoop_SetYaw(float yaw, uint32_t sample_ms, bool valid)
{
  uint32_t key = ControlAtomic_Enter();
  s_yaw.yaw = yaw; s_yaw.ms = sample_ms; s_yaw.valid = valid;
  ControlAtomic_Exit(key);
}

void ControlLoop_Tick(uint32_t now_ms)
{
  if (!s_ready) return;
  uint32_t gap = now_ms - s_last_tick;
  if (s_seen_tick && gap < CONTROL_PERIOD_MS) return;
  if (s_seen_tick && gap > g_control_max_gap_ms) g_control_max_gap_ms = gap;
  s_seen_tick = true;
  s_last_tick = now_ms;
#if defined(__arm__) || defined(__thumb__)
  uint32_t begin_cycles = DWT->CYCCNT;
#endif
  /* The mailbox cannot be written by foreground while SysTick executes. */
  LineFollow_SetYaw(s_yaw.yaw, s_yaw.ms, now_ms, s_yaw.valid);
  EncoderCounts counts = Encoder_Read();
  WheelSpeed_Sample(counts.left, counts.right, now_ms, g_motor_output_enabled);
  VehicleControl_Update();
#if defined(__arm__) || defined(__thumb__)
  uint32_t cycles = DWT->CYCCNT - begin_cycles;
  if (cycles > g_control_max_cycles) g_control_max_cycles = cycles;
  if (cycles >= SystemCoreClock / 1000U) ++g_control_overrun_count;
#endif
}
