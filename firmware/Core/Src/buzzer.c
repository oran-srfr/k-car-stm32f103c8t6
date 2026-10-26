#include "buzzer.h"
#include "buzzer_port.h"

#define QUEUE_SIZE 8U

typedef struct { uint16_t hz; uint16_t ms; } Tone;
typedef struct { const Tone *tones; uint8_t count; } Pattern;

static const Tone point_b[] = { {2400, 120} };
static const Tone point_c[] = { {1800, 100}, {0, 80}, {1800, 100} };
static const Tone obstacle[] = {
    {3000, 120}, {0, 80}, {3000, 120}, {0, 80}, {3000, 120}
};
static const Tone leave_line[] = { {1200, 250} };
static const Pattern patterns[BUZZER_EVENT_COUNT] = {
    {point_b, sizeof(point_b) / sizeof(point_b[0])},
    {point_c, sizeof(point_c) / sizeof(point_c[0])},
    {obstacle, sizeof(obstacle) / sizeof(obstacle[0])},
    {leave_line, sizeof(leave_line) / sizeof(leave_line[0])}
};

static bool ready;
static bool playing;
static BuzzerEvent current;
static BuzzerEvent queue[QUEUE_SIZE];
static uint8_t head;
static uint8_t count;
static uint8_t tone_index;
static uint32_t tone_start_ms;

static void start(BuzzerEvent event, uint32_t now_ms)
{
    current = event;
    tone_index = 0;
    tone_start_ms = now_ms;
    playing = true;
    BuzzerHw_SetFrequency(patterns[event].tones[0].hz);
}

bool Buzzer_Init(void)
{
    head = count = tone_index = 0;
    playing = false;
    ready = BuzzerHw_Init();
    if (ready) { BuzzerHw_SetFrequency(0); }
    return ready;
}

bool Buzzer_Play(BuzzerEvent event, uint32_t now_ms)
{
    if (!ready || (unsigned)event >= BUZZER_EVENT_COUNT) { return false; }
    if (event == BUZZER_OBSTACLE) {
        /* Coalesce a repeated alert without restarting an already playing alert.
         * New obstacle alerts preempt and discard stale normal notifications. */
        if (playing && current == BUZZER_OBSTACLE) { return true; }
        head = count = 0;
        start(event, now_ms);
    } else if (!playing) {
        start(event, now_ms);
    } else {
        if (count == QUEUE_SIZE) { return false; }
        queue[(head + count) % QUEUE_SIZE] = event;
        ++count;
    }
    return true;
}

void Buzzer_Task(uint32_t now_ms)
{
    if (!ready || !playing) { return; }
    const Pattern *pattern = &patterns[current];
    if ((uint32_t)(now_ms - tone_start_ms) < pattern->tones[tone_index].ms) { return; }
    ++tone_index;
    if (tone_index < pattern->count) {
        /* Do not skip an audible segment or gap after a delayed loop iteration. */
        tone_start_ms = now_ms;
        BuzzerHw_SetFrequency(pattern->tones[tone_index].hz);
    } else if (count != 0) {
        BuzzerEvent next = queue[head];
        head = (head + 1U) % QUEUE_SIZE;
        --count;
        start(next, now_ms);
    } else {
        playing = false;
        BuzzerHw_SetFrequency(0);
    }
}

void Buzzer_Stop(void)
{
    head = count = 0;
    playing = false;
    if (ready) { BuzzerHw_SetFrequency(0); }
}

bool Buzzer_IsReady(void) { return ready; }
bool Buzzer_IsBusy(void) { return playing; }
