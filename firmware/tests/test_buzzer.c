#include "buzzer.h"
#include "buzzer_port.h"
#include <assert.h>
#include <stdio.h>

static bool hw_ok = true;
static uint16_t hz;
static unsigned changes;
bool BuzzerHw_Init(void) { hz = 0; return hw_ok; }
void BuzzerHw_SetFrequency(uint16_t value) { hz = value; ++changes; }

static void reset(void)
{
    hw_ok = true;
    assert(Buzzer_Init());
    assert(Buzzer_IsReady() && !Buzzer_IsBusy() && hz == 0);
    changes = 0;
}

static void test_patterns(void)
{
    reset();
    assert(Buzzer_Play(BUZZER_POINT_B, 0));
    assert(hz == 2400);
    Buzzer_Task(119);
    assert(hz == 2400);
    Buzzer_Task(120);
    assert(hz == 0 && !Buzzer_IsBusy());
    assert(Buzzer_Play(BUZZER_POINT_C, 200));
    assert(hz == 1800);
    Buzzer_Task(300);
    assert(hz == 0 && Buzzer_IsBusy());
    Buzzer_Task(379);
    assert(hz == 0);
    Buzzer_Task(380);
    assert(hz == 1800);
    Buzzer_Task(480);
    assert(hz == 0 && !Buzzer_IsBusy());
    assert(Buzzer_Play(BUZZER_LEAVE_LINE, 500));
    assert(hz == 1200);
    Buzzer_Task(749);
    assert(hz == 1200);
    Buzzer_Task(750);
    assert(!Buzzer_IsBusy() && hz == 0);
}

static void test_priority(void)
{
    reset();
    assert(Buzzer_Play(BUZZER_POINT_B, 0));
    assert(Buzzer_Play(BUZZER_POINT_C, 1)); /* Pending C will be discarded. */
    assert(Buzzer_Play(BUZZER_OBSTACLE, 10));
    assert(hz == 3000);
    unsigned before = changes;
    assert(Buzzer_Play(BUZZER_OBSTACLE, 100));
    assert(changes == before); /* Repeated alerts cannot restart or extend it. */
    assert(Buzzer_Play(BUZZER_LEAVE_LINE, 110)); /* Queued after the alert. */
    Buzzer_Task(130); assert(hz == 0);
    Buzzer_Task(210); assert(hz == 3000);
    Buzzer_Task(330); assert(hz == 0);
    Buzzer_Task(410); assert(hz == 3000);
    Buzzer_Task(530); assert(hz == 1200); /* Leave-line, not stale C. */
    Buzzer_Task(780); assert(hz == 0 && !Buzzer_IsBusy());
}

static void test_fifo_and_overflow(void)
{
    reset();
    assert(Buzzer_Play(BUZZER_POINT_B, 0));
    for (unsigned i = 0; i < 8; ++i) {
        assert(Buzzer_Play(BUZZER_LEAVE_LINE, 0));
    }
    assert(!Buzzer_Play(BUZZER_POINT_C, 0));
    Buzzer_Task(120);
    assert(hz == 1200);
    /* Drain and enqueue again to exercise the ring-buffer head wrap. */
    for (unsigned i = 1; i <= 8; ++i) {
        Buzzer_Task(120 + i * 250);
    }
    assert(!Buzzer_IsBusy() && hz == 0);
    assert(Buzzer_Play(BUZZER_POINT_B, 2200));
    assert(Buzzer_Play(BUZZER_LEAVE_LINE, 2200));
    assert(Buzzer_Play(BUZZER_POINT_C, 2200));
    Buzzer_Task(2320); assert(hz == 1200);
    Buzzer_Task(2570); assert(hz == 1800);
    Buzzer_Stop();
    assert(!Buzzer_IsBusy() && hz == 0);
    Buzzer_Task(10000);
    assert(hz == 0);
}

static void test_wrap_and_late_poll(void)
{
    reset();
    uint32_t start = UINT32_MAX - 49U;
    assert(Buzzer_Play(BUZZER_POINT_B, start));
    Buzzer_Task(69); assert(hz == 2400); /* 119 ms since start. */
    Buzzer_Task(70); assert(hz == 0 && !Buzzer_IsBusy());
    assert(Buzzer_Play(BUZZER_POINT_C, 100));
    Buzzer_Task(1000); assert(hz == 0 && Buzzer_IsBusy());
    Buzzer_Task(1079); assert(hz == 0);
    Buzzer_Task(1080); assert(hz == 1800); /* Gap was not skipped. */
    Buzzer_Task(1180); assert(hz == 0 && !Buzzer_IsBusy());
}

static void test_error_and_cancel(void)
{
    reset();
    assert(!Buzzer_Play((BuzzerEvent)-1, 0));
    assert(!Buzzer_Play(BUZZER_EVENT_COUNT, 0));
    assert(!Buzzer_IsBusy() && hz == 0);
    assert(Buzzer_Play(BUZZER_POINT_C, 0));
    assert(Buzzer_Play(BUZZER_POINT_B, 0));
    Buzzer_Stop();
    Buzzer_Task(5000);
    assert(hz == 0 && !Buzzer_IsBusy());
    assert(Buzzer_Play(BUZZER_LEAVE_LINE, 5001));
    assert(Buzzer_Init()); /* Reinitialization also cancels all audio. */
    assert(hz == 0 && !Buzzer_IsBusy());
    hw_ok = false;
    assert(!Buzzer_Init() && !Buzzer_IsReady());
    assert(!Buzzer_Play(BUZZER_OBSTACLE, 6000));
    Buzzer_Task(7000);
    Buzzer_Stop();
    assert(hz == 0 && !Buzzer_IsBusy());
}

int main(void)
{
    test_patterns();
    test_priority();
    test_fifo_and_overflow();
    test_wrap_and_late_poll();
    test_error_and_cancel();
    puts("PASS: 5 buzzer scenario groups");
    return 0;
}
