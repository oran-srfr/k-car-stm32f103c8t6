#ifndef K_CAR_APP_H
#define K_CAR_APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { APP_MODE_LAPS, APP_MODE_SIDE_PARK, APP_MODE_REVERSE_PARK } AppMode;
typedef enum {
    APP_WAIT_START, APP_FOLLOW_LINE, APP_OBSTACLE_HOLD,
    APP_PARKING, APP_FINISHED, APP_FAULT
} AppState;
/* Geometric waypoints, not printed letters: the arena has no A/B/C/D marks. */
typedef enum { APP_POINT_NONE, APP_POINT_A, APP_POINT_B, APP_POINT_C, APP_POINT_D } AppPoint;
typedef enum { APP_PARK_BUSY, APP_PARK_DONE, APP_PARK_FAILED } AppParkResult;
typedef enum { APP_DRIVE_STOP, APP_DRIVE_FOLLOW, APP_DRIVE_SIDE, APP_DRIVE_REVERSE } AppDrive;
typedef enum { APP_SOUND_B, APP_SOUND_C, APP_SOUND_OBSTACLE, APP_SOUND_LEAVE_LINE } AppSound;
typedef enum { APP_ERROR_NONE, APP_ERROR_LINE_LOST, APP_ERROR_PARK_FAILED,
               APP_ERROR_PARK_TIMEOUT, APP_ERROR_PARK_SENSOR } AppError;

/* Produced once per foreground iteration. Events must be consumed once, not levels. */
typedef struct {
    bool start_pressed;
    bool stop_pressed;
    AppMode selected_mode;
    bool line_valid;
    bool sonar_valid;
    uint16_t distance_mm;
    uint32_t sonar_sample_ms;
    AppPoint point_event;
    bool parking_space_event;
    bool left_line_event;
    AppParkResult park_result;
} AppInputs;

typedef struct {
    AppState state;
    AppMode mode;
    AppError error;
    uint8_t laps;
    uint32_t lap_ms[2];
    uint16_t obstacles;
    uint16_t side_spaces;
    uint16_t reverse_spaces;
} AppStatus;

void App_Init(uint32_t now_ms);
void App_Task(uint32_t now_ms);
/* Call from the main loop only; HMI receives a copy via Modules_Display(). */
AppStatus App_GetStatus(void);

#endif
