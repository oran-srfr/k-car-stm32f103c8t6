#include "wheel_speed.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static LineFollowResult_t line;
static const MotorCommandConfig_t config = {580.0f, 930.0f, 400.0f, 0.0f};
static uint32_t ms, left_count, right_count;
static float left, right;
static void sample(unsigned dl, unsigned dr)
{
  ms += 100U; left_count += dl; right_count += dr;
  WheelSpeed_Sample(left_count, right_count, ms, true);
  left = right = 580.0f;
  WheelSpeed_Apply(&line, &config, &left, &right);
}
static void reset(void)
{
  WheelSpeed_Init();
  line = (LineFollowResult_t){0};
  line.state = LINE_FOLLOW_TRACKING;
  line.control.track.status = LINE_TRACK_STATUS_NORMAL;
  ms = left_count = right_count = 0U;
  WheelSpeed_Sample(0U, 0U, ms, true);
}
static float plant(bool enabled, float deadzone, float lag)
{
  reset();
  g_wheel_speed_enabled = enabled;
  float vl = 0.0f, vr = 0.0f, edges_l = 0.0f, edges_r = 0.0f, error = 0.0f;
  bool active_seen = false;
  for (unsigned i = 0U; i < 16000U; ++i) {
    WheelSpeed_Sample((uint32_t)edges_l, (uint32_t)edges_r, i, true);
    float l = 580.0f, r = 580.0f;
    WheelSpeed_Apply(&line, &config, &l, &r);
    active_seen |= g_wheel_speed_status.active;
    assert(l >= 565.0f && r >= 565.0f && l + r <= 1160.01f);
    vl += 0.001f / lag * (1000.0f * (l - deadzone) / (580.0f - deadzone) * 0.97f - vl);
    vr += 0.001f / lag * (1000.0f * (r - deadzone) / (580.0f - deadzone) * 1.03f - vr);
    edges_l += 0.001f * vl; edges_r += 0.001f * vr;
    if (i >= 12000U) error += fabsf(vl - vr);
  }
  if (enabled) assert(active_seen);
  return error / 4000.0f;
}
int main(void)
{
  reset();
  for (unsigned i = 0U; i < 100U; ++i) sample(0U, 0U);
  assert(!g_wheel_speed_status.calibrated && !g_wheel_speed_status.active);
  assert(left == 580.0f && right == 580.0f); /* absent encoder is never a reason to stop/boost */
  reset();
  for (unsigned i = 0U; i < 2U; ++i) sample(100U, 110U);
  assert(!g_wheel_speed_status.calibrated);
  sample(100U, 110U);
  assert(g_wheel_speed_status.calibrated && g_wheel_speed_status.active);
  assert(fabsf(g_wheel_speed_status.reference_hz - 1050.0f) < 0.1f);
  assert(left > right && (left + right) * 0.5f <= 580.0f);
  for (unsigned i = 0U; i < 100U; ++i) sample(50U, 170U);
  assert(left <= 595.0f && right >= 565.0f); /* independent PI states stay bounded */
  sample(0U, 170U);
  assert(!g_wheel_speed_status.active && left == 580.0f && right == 580.0f);
  sample(100U, 110U);
  assert(g_wheel_speed_status.active);
  line.state = LINE_FOLLOW_TURNING;
  left = 0.0f; right = 800.0f;
  WheelSpeed_Apply(&line, &config, &left, &right);
  assert(left == 0.0f && right == 800.0f && !g_wheel_speed_status.active);
  line.state = LINE_FOLLOW_TRACKING;
  left = -100.0f; right = 580.0f;
  WheelSpeed_Apply(&line, &config, &left, &right);
  assert(left == -100.0f && right == 580.0f); /* A-only feedback is forward-only */
  g_wheel_speed_enabled = false;
  sample(100U, 110U);
  assert(left == 580.0f && right == 580.0f && !g_wheel_speed_status.active);
  g_wheel_speed_enabled = true;
  WheelSpeed_Sample(left_count, right_count, ms, false);
  left = right = 580.0f;
  WheelSpeed_Apply(&line, &config, &left, &right);
  assert(!g_wheel_speed_status.active && left == 580.0f && right == 580.0f);
  /* Time and monotonically counted pulse wrap remain well defined. */
  reset();
  ms = UINT32_MAX - 50U; left_count = right_count = UINT32_MAX - 50U;
  WheelSpeed_Sample(left_count, right_count, ms, false);
  WheelSpeed_Sample(left_count, right_count, ms, true);
  for (unsigned i = 0U; i < 3U; ++i) sample(100U, 100U);
  assert(g_wheel_speed_status.calibrated && g_wheel_speed_status.active);
  assert(left == 580.0f && right == 580.0f);
  for (unsigned dz = 0U; dz < 3U; ++dz) {
    float deadzone = dz == 0U ? 0.0f : (dz == 1U ? 450.0f : 550.0f);
    for (unsigned delay = 40U; delay <= 120U; delay += 40U) {
      float open = plant(false, deadzone, (float)delay * 0.001f);
      float closed = plant(true, deadzone, (float)delay * 0.001f);
      printf("Wheel speed plant: deadzone %.0f lag %u ms mismatch %.2f -> %.2f pulses/s\n",
             deadzone, delay, open, closed);
      assert(closed < 0.8f * open);
    }
  }
  g_wheel_speed_enabled = true;
  puts("PASS: forward speed calibration, independent PI, output caps, missing counts, bypass and wraps");
  return 0;
}
