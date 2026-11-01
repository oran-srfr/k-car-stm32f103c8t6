#include "wheel_speed.h"
#include <math.h>

/* A-only encoders provide unsigned pulse frequency. Learn the reference at
 * the existing straight duty, so no wheel diameter/PPR/speed is invented. */
volatile bool g_wheel_speed_enabled = WHEEL_SPEED_ENABLED_DEFAULT != 0U;
volatile WheelSpeedStatus g_wheel_speed_status;
static uint32_t s_ms, s_left, s_right, s_sequence, s_used_sequence;
static bool s_sample_started, s_valid;
static bool s_have_rates;
static uint8_t s_learned;
static float s_reference_sum, s_last_learn_rate, s_integral[2], s_dt;

static float Clamp(float value, float limit)
{
  return fmaxf(-limit, fminf(value, limit));
}

void WheelSpeed_Init(void)
{
  s_sample_started = s_valid = false;
  s_have_rates = false;
  s_sequence = s_used_sequence = 0U;
  s_learned = 0U;
  s_reference_sum = s_integral[0] = s_integral[1] = 0.0f;
  s_last_learn_rate = 0.0f;
  g_wheel_speed_status = (WheelSpeedStatus){0};
}

void WheelSpeed_Sample(uint32_t left, uint32_t right, uint32_t now_ms, bool moving)
{
  if (!moving || !s_sample_started) {
    s_left = left; s_right = right; s_ms = now_ms;
    s_sample_started = moving;
    s_valid = false;
    s_have_rates = false;
    g_wheel_speed_status.active = false;
    s_integral[0] = s_integral[1] = 0.0f;
    if (!g_wheel_speed_status.calibrated) {
      s_learned = 0U; s_reference_sum = s_last_learn_rate = 0.0f;
    }
    return;
  }
  uint32_t elapsed = now_ms - s_ms;
  if (elapsed < WHEEL_SPEED_WINDOW_MS) return;
  uint32_t dl = left - s_left, dr = right - s_right;
  s_left = left; s_right = right; s_ms = now_ms;
  s_valid = elapsed <= 2U * WHEEL_SPEED_WINDOW_MS && dl >= WHEEL_SPEED_MIN_EDGES && dr >= WHEEL_SPEED_MIN_EDGES &&
            dl < 10000U && dr < 10000U;
  float left_hz = (float)dl * 1000.0f / (float)elapsed;
  float right_hz = (float)dr * 1000.0f / (float)elapsed;
  if (s_valid && s_have_rates) {
    left_hz = 0.5f * (left_hz + g_wheel_speed_status.left_hz);
    right_hz = 0.5f * (right_hz + g_wheel_speed_status.right_hz);
  }
  s_have_rates = s_valid;
  g_wheel_speed_status.left_hz = left_hz;
  g_wheel_speed_status.right_hz = right_hz;
  s_dt = (float)elapsed * 0.001f;
  ++s_sequence;
}

void WheelSpeed_Apply(const LineFollowResult_t *follow, const MotorCommandConfig_t *config,
                      float *left, float *right)
{
  g_wheel_speed_status.active = false;
  g_wheel_speed_status.left_correction = g_wheel_speed_status.right_correction = 0.0f;
  bool eligible = g_wheel_speed_enabled && s_valid && follow->state == LINE_FOLLOW_TRACKING &&
                  follow->fault == LINE_FOLLOW_FAULT_NONE &&
                  follow->control.track.status == LINE_TRACK_STATUS_NORMAL &&
                  fabsf(follow->control.track.deviation) <= 1.0f &&
                  *left >= 480.0f && *right >= 480.0f &&
                  *left <= 680.0f && *right <= 680.0f && config->base_speed >= 480.0f;
  bool new_sample = s_sequence != s_used_sequence;
  s_used_sequence = s_sequence;
  if (!eligible) {
    s_integral[0] = s_integral[1] = 0.0f;
    if (!g_wheel_speed_status.calibrated) {
      s_learned = 0U; s_reference_sum = s_last_learn_rate = 0.0f;
    }
    return;
  }
  float rates[2] = {g_wheel_speed_status.left_hz, g_wheel_speed_status.right_hz};
  if (!g_wheel_speed_status.calibrated) {
    if (new_sample && fabsf(*left - *right) <= 50.0f &&
        (*left + *right) * 0.5f >= config->base_speed - 10.0f &&
        rates[0] <= 1.5f * rates[1] && rates[1] <= 1.5f * rates[0]) {
      float mean_rate = 0.5f * (rates[0] + rates[1]);
      float tolerance = fmaxf(WHEEL_SPEED_LEARN_TOLERANCE * s_last_learn_rate,
                              1000.0f / (float)WHEEL_SPEED_WINDOW_MS);
      if (s_last_learn_rate > 0.0f && fabsf(mean_rate - s_last_learn_rate) > tolerance) {
        s_learned = 0U; s_reference_sum = 0.0f;
      }
      s_last_learn_rate = mean_rate;
      s_reference_sum += mean_rate;
      if (++s_learned >= WHEEL_SPEED_LEARN_WINDOWS) {
        g_wheel_speed_status.reference_hz = s_reference_sum / (float)s_learned;
        g_wheel_speed_status.calibrated = true;
      }
    } else if (new_sample) {
      s_learned = 0U;
      s_reference_sum = s_last_learn_rate = 0.0f;
    }
    if (!g_wheel_speed_status.calibrated) return;
  }
  float commands[2] = {*left, *right}, correction[2];
  float reference = g_wheel_speed_status.reference_hz;
  if (!isfinite(reference) || reference <= 0.0f) return;
  for (unsigned i = 0U; i < 2U; ++i) {
    float error = commands[i] / config->base_speed - rates[i] / reference;
    if (new_sample) s_integral[i] = Clamp(s_integral[i] + WHEEL_SPEED_KI * s_dt * error, WHEEL_SPEED_MAX_CORRECTION);
    correction[i] = Clamp(WHEEL_SPEED_KP * error + s_integral[i], WHEEL_SPEED_MAX_CORRECTION);
  }
  /* Keep original mean duty as an upper bound. Physical speed still depends
   * on measured motor response; this cap does not assume PWM is linear. */
  float common = fmaxf(0.0f, 0.5f * (correction[0] + correction[1]));
  *left = fmaxf(0.0f, fminf(*left + correction[0] - common, config->max_forward_command));
  *right = fmaxf(0.0f, fminf(*right + correction[1] - common, config->max_forward_command));
  g_wheel_speed_status.left_correction = *left - commands[0];
  g_wheel_speed_status.right_correction = *right - commands[1];
  g_wheel_speed_status.active = true;
}
