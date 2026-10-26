#include "ultrasonic.h"
#include "ultrasonic_port.h"

#define SAMPLE_PERIOD_MS 60U
#define ECHO_TIMEOUT_MS 30U /* Less than the 65.536 ms counter wrap period. */
#define MIN_DISTANCE_MM 20U
#define MAX_DISTANCE_MM 4000U

typedef enum { IDLE, TRIGGER, WAIT_RISE, WAIT_FALL, COMPLETE } Phase;
static volatile Phase phase;
static volatile uint16_t rise_us;
static volatile uint16_t width_us;
static volatile uint32_t edge_ms;
static uint32_t cycle_ms;
static bool ready;
static UlrReading reading;

bool Ulr_Init(uint32_t now_ms)
{
    uint32_t key = UlrHw_Lock();
    ready = false;
    phase = IDLE;
    rise_us = width_us = 0U;
    edge_ms = now_ms;
    cycle_ms = now_ms - SAMPLE_PERIOD_MS;
    reading = (UlrReading){ .status = ULR_NOT_READY, .sample_ms = now_ms };
    ready = UlrHw_Init();
    if (!ready) { reading.status = ULR_HW_ERROR; }
    UlrHw_Unlock(key);
    return ready;
}

void Ulr_OnTriggerEnd(void)
{
    if (phase == TRIGGER) { phase = WAIT_RISE; }
}

void Ulr_OnEchoEdge(bool high, uint16_t counter_us, uint32_t now_ms)
{
    /* Only capture timestamps here. Conversion/publication stay in the loop. */
    if ((uint32_t)(now_ms - cycle_ms) >= ECHO_TIMEOUT_MS) { return; }
    if (high && phase == WAIT_RISE) {
        rise_us = counter_us;
        phase = WAIT_FALL;
    } else if (!high && phase == WAIT_FALL) {
        width_us = (uint16_t)(counter_us - rise_us);
        edge_ms = now_ms;
        phase = COMPLETE;
    }
}

void Ulr_Task(uint32_t now_ms)
{
    uint32_t key = UlrHw_Lock();
    if (!ready) { UlrHw_Unlock(key); return; }

    if (phase == COMPLETE) {
        uint16_t pulse = width_us;
        /* 343 m/s at about 20 C: d(mm) = Echo high time(us) * 343 / 2000. */
        uint32_t mm = ((uint32_t)pulse * 343U + 1000U) / 2000U;
        UlrStatus result = ULR_OK;
        if (mm < MIN_DISTANCE_MM) { result = ULR_TOO_CLOSE; }
        else if (mm > MAX_DISTANCE_MM) { result = ULR_TOO_FAR; }
        reading = (UlrReading){ .status = result, .distance_mm = (uint16_t)mm,
                                .pulse_us = pulse, .sample_ms = edge_ms };
        phase = IDLE;
    } else if (phase != IDLE && (uint32_t)(now_ms - cycle_ms) >= ECHO_TIMEOUT_MS) {
        UlrHw_CancelTrigger();
        phase = IDLE;
        reading = (UlrReading){ .status = ULR_NO_ECHO, .sample_ms = now_ms };
    }

    if (phase == IDLE && (uint32_t)(now_ms - cycle_ms) >= SAMPLE_PERIOD_MS) {
        cycle_ms = now_ms;
        if (UlrHw_EchoHigh()) {
            reading = (UlrReading){ .status = ULR_ECHO_STUCK, .sample_ms = now_ms };
        } else {
            phase = TRIGGER;
            UlrHw_Trigger();
        }
    }
    UlrHw_Unlock(key);
}

UlrReading Ulr_GetReading(void)
{
    uint32_t key = UlrHw_Lock();
    UlrReading result = reading;
    UlrHw_Unlock(key);
    return result;
}
