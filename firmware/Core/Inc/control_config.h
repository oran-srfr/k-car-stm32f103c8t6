#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制参数默认值集中在这里，运行时仍可通过 Watch 修改对应配置变量。 */
#define CONTROL_PERIOD_MS                    (1U)
#define CONTROL_PERIOD_SECONDS               ((float)CONTROL_PERIOD_MS / 1000.0f)
#define CONTROL_KP_DEFAULT                   (80.0f)
#define CONTROL_KD_DEFAULT                   (0.3f)
#define CONTROL_PD_OUTPUT_LIMIT_DEFAULT      (120.0f)
#define CONTROL_PD_DERIVATIVE_ALPHA          (0.25f)
#define CONTROL_PD_D_TERM_LIMIT              (30.0f)
/* Driver units: 1000 = full duty. 650 = 65%, NOT TIM3 CCR=650.
 * Direct correction uses 45%..77%; normal driving remains around 65%. */
#define CONTROL_BASE_SPEED_DEFAULT           (650.0f)
#define CONTROL_MIN_FORWARD_COMMAND_DEFAULT  (550.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (770.0f)
#define CONTROL_STRAIGHT_TRIM_DEFAULT        (0.0f)

/* 两轮始终前进，以差速跟随连续圆弧，不用定时原地旋转。 */
#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (630.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (620.0f)
#define LINE_FOLLOW_SEARCH_SPEED_DEFAULT            (650.0f)
#define LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT   (0.5f)
#define LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT      (1.5f)
#define LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT       (120.0f)
#define LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT          (1000U)
#define LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT         (10U)
#define LINE_FOLLOW_CURVE_HOLD_CYCLES                (20U)
#define LINE_FOLLOW_BASE_STEP                       (10.0f)
/* 同方向偏差持续 30 ms，或已到外探头时进入强修正。
 * 45% 内轮下限只在强修正中启用；普通循迹仍保持 55% 下限。 */
#define LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES         (3U)
#define LINE_FOLLOW_HARD_TURN_DEVIATION              (1.5f)
#define LINE_FOLLOW_HARD_TURN_MIN_FORWARD            (450.0f)
#define LINE_FOLLOW_HARD_TURN_GAIN                   (1.25f)
/* Direct four-sensor recovery uses a two-wheel differential at all times.
 * Inner correction runs 450/700; outer correction uses a wider 450/770.
 * Four indicators off first drive straight at 550 for 80 ms, then search. */
#define LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT       (700.0f)
#define LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT       (450.0f)
#define LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT       (770.0f)
#define LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT       (450.0f)
#define LINE_FOLLOW_ALL_OFF_SLOW_CYCLES              (80U)
#ifndef LINE_FOLLOW_DISCRETE_MODE
#define LINE_FOLLOW_DISCRETE_MODE                    (1U)
#endif

#endif /* CONTROL_CONFIG_H */
