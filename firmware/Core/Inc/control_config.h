#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制参数默认值集中在这里，运行时仍可通过 Watch 修改对应配置变量。 */
#define CONTROL_PERIOD_MS                    (10U)
#define CONTROL_PERIOD_SECONDS               ((float)CONTROL_PERIOD_MS / 1000.0f)
#define CONTROL_KP_DEFAULT                   (40.0f)
#define CONTROL_KD_DEFAULT                   (0.2f)
#define CONTROL_PD_OUTPUT_LIMIT_DEFAULT      (120.0f)
#define CONTROL_PD_DERIVATIVE_ALPHA          (0.25f)
#define CONTROL_PD_D_TERM_LIMIT              (30.0f)
/* Driver units: 1000 = full duty. 650 = 65%, NOT TIM3 CCR=650.
 * User requested 50..80% duty; use 55% floor with a 5-point margin. */
#define CONTROL_BASE_SPEED_DEFAULT           (650.0f)
#define CONTROL_MIN_FORWARD_COMMAND_DEFAULT  (550.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (800.0f)
#define CONTROL_STRAIGHT_TRIM_DEFAULT        (0.0f)

/* 两轮始终前进，以差速跟随连续圆弧，不用定时原地旋转。 */
#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (630.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (620.0f)
#define LINE_FOLLOW_SEARCH_SPEED_DEFAULT            (650.0f)
#define LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT   (0.5f)
#define LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT      (1.5f)
#define LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT       (120.0f)
#define LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT          (30U)
#define LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT         (10U)
#define LINE_FOLLOW_CURVE_HOLD_CYCLES                (20U)
#define LINE_FOLLOW_BASE_STEP                       (10.0f)

#endif /* CONTROL_CONFIG_H */
