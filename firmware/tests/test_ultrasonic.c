#include "ultrasonic.h"
#include "ultrasonic_port.h"
#include <assert.h>
#include <stdio.h>

static bool hw_ok = true;
static bool echo_high;
static bool trig_high;
static unsigned triggers;
static unsigned cancels;
static unsigned locked;

bool UlrHw_Init(void) { trig_high = false; return hw_ok; }
uint16_t UlrHw_Counter(void) { return 0; }
bool UlrHw_EchoHigh(void) { return echo_high; }
void UlrHw_Trigger(void) { assert(locked); ++triggers; trig_high = true; }
void UlrHw_CancelTrigger(void) { trig_high = false; ++cancels; }
uint32_t UlrHw_Lock(void) { return locked++; }
void UlrHw_Unlock(uint32_t key) { assert(locked == key + 1); locked = key; }

static void setup(uint32_t now)
{
    hw_ok = true;
    echo_high = false;
    triggers = cancels = 0;
    assert(Ulr_Init(now));
    assert(Ulr_GetReading().status == ULR_NOT_READY);
    Ulr_Task(now);
    assert(triggers == 1 && trig_high);
}

static void trigger_end(void)
{
    trig_high = false; /* Hardware IRQ lowers the pin before calling the core. */
    Ulr_OnTriggerEnd();
}

static UlrReading measure(uint16_t pulse)
{
    setup(100);
    trigger_end();
    Ulr_OnEchoEdge(true, 200, 101);
    Ulr_OnEchoEdge(false, (uint16_t)(200 + pulse), 125);
    Ulr_Task(125);
    return Ulr_GetReading();
}

static void test_valid_and_period(void)
{
    UlrReading r = measure(5831); /* 1000 mm at nominal sound speed. */
    assert(r.status == ULR_OK && r.distance_mm == 1000 && r.pulse_us == 5831);
    assert(r.sample_ms == 125);
    Ulr_Task(159);
    assert(triggers == 1);
    Ulr_Task(160);
    assert(triggers == 2);
}

static void test_timeout_and_recovery(void)
{
    setup(0);
    trigger_end();
    Ulr_OnEchoEdge(true, 1000, 1); /* No falling edge. */
    Ulr_Task(29);
    assert(Ulr_GetReading().status == ULR_NOT_READY);
    Ulr_Task(30);
    assert(Ulr_GetReading().status == ULR_NO_ECHO && cancels == 1 && !trig_high);
    Ulr_OnEchoEdge(false, 2000, 31); /* Late echo cannot revive old reading. */
    Ulr_Task(60);
    trigger_end();
    Ulr_OnEchoEdge(true, 3000, 61);
    Ulr_OnEchoEdge(false, 4000, 62);
    Ulr_Task(62);
    assert(Ulr_GetReading().status == ULR_OK);
    /* Timeout of the next cycle must discard previously valid data. */
    Ulr_Task(120);
    trigger_end();
    Ulr_Task(150);
    assert(Ulr_GetReading().status == ULR_NO_ECHO);
    assert(Ulr_GetReading().distance_mm == 0);
}

static void test_range(void)
{
    assert(measure(466).distance_mm == 80); /* Competition stop threshold. */
    assert(measure(116).status == ULR_OK); /* Rounded 20 mm. */
    assert(measure(100).status == ULR_TOO_CLOSE);
    assert(measure(23323).status == ULR_OK);
    assert(measure(24000).status == ULR_TOO_FAR);
    setup(0);
    /* Even a missing trigger-end interrupt must not hang the foreground task. */
    Ulr_Task(30);
    assert(Ulr_GetReading().status == ULR_NO_ECHO && !trig_high);
}

static void test_stuck_and_hw_failure(void)
{
    setup(0);
    trigger_end();
    Ulr_Task(30);
    echo_high = true;
    Ulr_Task(60);
    assert(Ulr_GetReading().status == ULR_ECHO_STUCK && triggers == 1);
    echo_high = false;
    Ulr_Task(120);
    assert(triggers == 2);
    hw_ok = false;
    assert(!Ulr_Init(200));
    Ulr_Task(500);
    assert(Ulr_GetReading().status == ULR_HW_ERROR);
    assert(triggers == 2);
}

static void test_wrap_and_noise(void)
{
    uint32_t start = UINT32_MAX - 10U;
    setup(start);
    /* Echo edges before trigger end and falling before rising are ignored. */
    Ulr_OnEchoEdge(true, 65000, start);
    trigger_end();
    Ulr_OnEchoEdge(false, 65001, start + 1);
    Ulr_OnEchoEdge(true, 65000, start + 2);
    Ulr_OnEchoEdge(true, 65500, start + 3); /* Duplicate rising edge ignored. */
    Ulr_OnEchoEdge(false, 464, 1); /* 1000 us across 16-bit timer wrap. */
    Ulr_OnEchoEdge(true, 500, 2);
    Ulr_OnEchoEdge(false, 600, 3); /* Extra pulse cannot overwrite COMPLETE. */
    Ulr_Task(4);
    UlrReading r = Ulr_GetReading();
    assert(r.status == ULR_OK && r.pulse_us == 1000 && r.distance_mm == 172);
    assert(r.sample_ms == 1);
    Ulr_Task(49); /* 60 ms across millisecond wrap. */
    assert(triggers == 2);
    trigger_end();
    Ulr_OnEchoEdge(true, 1000, 79); /* Exactly 30 ms is too late. */
    Ulr_OnEchoEdge(false, 2000, 80);
    Ulr_Task(80);
    assert(Ulr_GetReading().status == ULR_NO_ECHO);
}

int main(void)
{
    test_valid_and_period();
    test_timeout_and_recovery();
    test_range();
    test_stuck_and_hw_failure();
    test_wrap_and_noise();
    assert(locked == 0);
    puts("PASS: 5 ultrasonic scenario groups");
    return 0;
}
