#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制参数默认值集中在这里，运行时仍可通过 Watch 修改对应配置变量。 */
#define CONTROL_PERIOD_MS                    (10U)
#define CONTROL_PERIOD_SECONDS               ((float)CONTROL_PERIOD_MS / 1000.0f)
#define CONTROL_KP_DEFAULT                   (1.0f)
#define CONTROL_KD_DEFAULT                   (0.0f)
#define CONTROL_PD_OUTPUT_LIMIT_DEFAULT      (10.0f)
#define CONTROL_BASE_SPEED_DEFAULT           (300.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (800.0f)

/* 以下速度均为抽象软件命令，需等实车到手后重新调节。 */
#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (250.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (200.0f)
#define LINE_FOLLOW_SEARCH_SPEED_DEFAULT             (120.0f)
#define LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT   (0.5f)
#define LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT      (1.5f)
#define LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT        (4.0f)
#define LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT          (30U)
#define LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT         (10U)

#endif /* CONTROL_CONFIG_H */
