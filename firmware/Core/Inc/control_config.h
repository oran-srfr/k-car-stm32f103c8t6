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
 * 2026-10 修正①：原来 550~770 只有 110 headroom，最大差速 22%，
 *   按 15cm 轮距估算最小转弯半径约 45cm，急弯转不过来 → 放开到 400~930。
 * 2026-10 修正②（灵敏度）：直行基准 650 → 580。
 *   速度越低，同样的控制延迟内走过的距离越短、积累的横向偏差越小，主观上更「跟手」。
 *   嫌整体太慢可以把 580 往回调，但调高会重新变「迟钝」，两者要权衡。 */
#define CONTROL_BASE_SPEED_DEFAULT           (580.0f)
#define CONTROL_MIN_FORWARD_COMMAND_DEFAULT  (400.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (930.0f)
#define CONTROL_STRAIGHT_TRIM_DEFAULT        (0.0f)

/* ★ 级联约束：curve_speed 必须 ≤ base_speed，sharp_curve_speed 必须 ≤ curve_speed。
 *   否则 LineFollow_ConfigIsValid() 判非法 → 直接进故障、电机清零，表现为「跑一下就停」。
 *   改 base 时这三行要一起改。 */
#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (560.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (540.0f)
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
/* Discrete steering: the INNER pair sets the sustained correction limit.
 * Short inner-probe errors use the gentle ramp below; outer probes bypass
 * that ramp and retain the existing sharp-turn detection and recovery. */
#define LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT       (680.0f)
#define LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT       (480.0f)
/* Single-inner corrections start gently. Only a persistent same-side error
 * ramps to half the INNER_FAST/INNER_SLOW difference (100 by default).
 * Center, direction reversal, outer probes and recovery clear the ramp. */
#define LINE_FOLLOW_INNER_INITIAL_CORRECTION         (25.0f)
#define LINE_FOLLOW_INNER_CORRECTION_STEP             (1.0f)
#define LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT       (940.0f)
#define LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT       (320.0f)
/* 四灯全灭时先「直行」多少个控制周期再开始差速搜索。
 * 80 周期 ≈ 80 ms —— 丢线后还要直冲 80 ms 才肯找线，弯道上这就是「反应慢」。
 * 收到 25 周期（≈25 ms），一丢线几乎立刻开始朝最后方向找。 */
#define LINE_FOLLOW_ALL_OFF_SLOW_CYCLES              (25U)
/* Sharp turns: stop the inside wheel, drive the outside wheel forward.
 * Confirm an outer probe for 6 ms; a three-probe corner pattern or an
 * outer-probe -> lost transition starts the turn immediately. */
#define LINE_FOLLOW_TURN_COMMAND                     (600.0f)
#define LINE_FOLLOW_TURN_TRIGGER_CYCLES               (6U)
#define LINE_FOLLOW_TURN_MIN_CYCLES                   (60U)
#define LINE_FOLLOW_TURN_MAX_CYCLES                   (1200U)
#define LINE_FOLLOW_TURN_ALL_BLACK_CYCLES             (150U)
#ifndef LINE_FOLLOW_DISCRETE_MODE
#define LINE_FOLLOW_DISCRETE_MODE                    (1U)
#endif

#endif /* CONTROL_CONFIG_H */
