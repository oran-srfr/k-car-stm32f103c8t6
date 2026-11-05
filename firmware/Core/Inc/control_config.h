#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 控制参数默认值集中在这里，运行时仍可通过 Watch 修改对应配置变量。 */
#define CONTROL_PERIOD_MS                    (1U)
#define CONTROL_PERIOD_SECONDS               ((float)CONTROL_PERIOD_MS / 1000.0f)

/* ===== 2026-10-30 第六轮：切回连续 PD（离散开关量到顶了）=====
 * 现象：差速从 450 一路降到 240、加了 6ms 去抖、基速降到 520，
 *       直线摆头仍在，且「出弧线之后最明显」。
 * 判断：出弧线后车身带残余角速度往外甩，开关量只有"修正/不修正"两档，
 *       没有"反向阻尼"，所以必然以极限环形式来回摆 —— 这是结构问题，参数治不了。
 *
 * PD 的三档（偏差加权值 ±1 / ±2 / ±3）：
 *   |dev|=1 → c = 100 → 轮速 600/400  差速 200  半径 37cm  （直线小幅偏，轻修）
 *   |dev|=2 → c = 200 → 轮速 800/400  差速 400  半径 22cm
 *   |dev|=3 → c = 265 → 轮速 930/400  差速 530  半径 19cm  （外探头，直角/急弯）
 *
 * 注意 D 项的量纲：代码里 derivative = Δdev / 0.001s，一次 1 单位跳变得到 1000，
 *   经 alpha=0.25 低通后首拍约 250。所以 Kd 给 0.15 时 D 项首拍 ≈ 37。
 *
 * ★ 为什么 Kd 不大：出弧线后的摆头是低频的（1~2 Hz，偏差每 100 ms 才变一格），
 *   此时 Δdev/Δt 只有 10~20，D 项算出来不到 1 —— 对低频摆动基本无效。
 *   给大了反而在"传感器跳变"这种高频事件上注入冲击，成为新的抖动源（量化噪声放大）。
 *   PD 治这个摆头靠的是「分级 P」＋「取消 1.25 倍强修正放大」，D 只做轻微缓冲。 */
/* ===== 2026-10-30 第八轮：直线修正再减弱，弧线靠"持续偏差放大"补回 =====
 * 需求：直线出线修正时两轮转速差再小一点。
 * 难点：直接降 Kp 会同时削弱弧线（第七轮已经撞过这个墙）。
 * 解法：用时间尺度区分两者 ——
 *   直线上的偏离是短暂的（修正完就回中，约 100~250 ms）
 *   弧线上的偏离是持续的（R60 半圆要跑几秒）
 * 于是：基础 Kp 降到 85（管直线），把"同向偏差持续"的触发时间从 50 ms 拉到
 *   200 ms、放大倍数从 1.25 提到 1.8（管弧线）。
 *   另外 |dev| >= 1.5（即压到外探头）仍然立即放大，不看时间 —— 那是真的跑偏了。
 *
 * 现在各级输出（基速 520 时）：
 *   |dev|=0              → 520/520   差速   0
 *   |dev|=1 短暂(直线)    → 550/380   差速 170   ← 本次从 220 降下来
 *   |dev|=1 持续>200ms    → 686/380   差速 306   （弧线，放大 1.8 倍）
 *   |dev|=2 (内+外)       → 930/450   差速 480   （立即放大，不等时间）
 *   |dev|=3 (单外探头)     → 930/450   差速 480
 *
 * 调法：
 *   直线还摆 → Kp 85 → 70（差速 140）
 *   弧线切内/跟不上 → HARD_TURN_GAIN 1.8 → 2.0，或 TRIGGER 200 → 150
 *   入弯瞬间冲出去 → TRIGGER 200 → 120（更早放大） */
/* ===== 2026-10-30 第十轮：关掉微分项（Kd 0.15 → 0）=====
 * 实测发现：两个内探头都压线（偏差 = 0）时，两轮转速仍然不一样。
 * 原因就是微分项：
 *   derivative = (dev - 上次) / 0.001 → 一次 1 格跳变就是 1000
 *   滤波后约 250，d_term = 0.15 × 250 = 37.5
 *   dev=0 时 output = Kp×0 + 37.5 = 37.5 → 轮速 570/470，差 21%
 *
 * 更严重的是：四路数字量只有 4 个档位，探头每次跳变都产生一次 ±37 的冲击，
 * 这些冲击与跳变同频，正好构成自激振荡 —— 这是持续摆头的主要来源之一。
 *
 * 结论：在量化输入上做微分，放大的是量化噪声而不是有效信息。
 *   Kd 置 0 后，|dev|=0 → 输出严格为 0 → 两轮严格同速。
 *   若以后需要阻尼，应该先对偏差做更长时间的低通（而不是直接对阶跃求导）。
 *
 * 另外：如果两轮命令完全相同时车仍然跑偏，那是电机/减速箱的个体差异（开环不对称），
 * 用 CONTROL_STRAIGHT_TRIM_DEFAULT 补偿（正数让左轮加快 = 向右修）。 */
/* ===== 2026-10-30 第十一轮：直线修正再降（Kp 85 → 60）=====
 * 需求：直线出线修正时两轮转速差再小一点（实测仍在摆头）。
 * 做法：Kp 85 → 60，同时把"同向偏差持续 200 ms"的放大倍数 1.8 → 2.5，
 *       让弧线上的力度保持不变。
 *
 * 各级输出（基速 520、内轮下限 320）：
 *   |dev|=0              → 520/520   差速   0
 *   |dev|=1 短暂(直线)    → 460/340   差速 120   ← 从 170 降下来（−29%）
 *   |dev|=1 持续>200ms    → 750/450   差速 300   （弧线，放大 2.5 倍）
 *   |dev|=2 或 3          → 930/450   差速 480   （立即放大，不等时间）
 *
 * 调法：
 *   直线还摆 → Kp 60 → 45（差速 90）
 *   弧线切内/跟不上 → HARD_TURN_GAIN 2.5 → 2.8，或 TRIGGER 200 → 150
 *   入弯瞬间冲出去 → TRIGGER 200 → 120（更早放大） */
#define CONTROL_KP_DEFAULT                   (60.0f)
#define CONTROL_KD_DEFAULT                   (0.0f)
#define CONTROL_PD_OUTPUT_LIMIT_DEFAULT      (265.0f)
#define CONTROL_PD_DERIVATIVE_ALPHA          (0.25f)
#define CONTROL_PD_D_TERM_LIMIT              (50.0f)
/* Driver units: 1000 = full duty. 650 = 65%, NOT TIM3 CCR=650.
 * 2026-10 修正①：原来 550~770 只有 110 headroom，最大差速 22%，
 *   按 15cm 轮距估算最小转弯半径约 45cm，急弯转不过来 → 放开到 380~930。
 * 2026-10 修正②（直行摆头）：基速 580 → 520。
 * 2026-10 修正③（第九轮）：降低"直线修正时"两轮的绝对转速，差速保持不变。
 *   原来 |dev|=1 输出 550/380（平均 465），现在 490/320（平均 405）——
 *   差速仍是 170，但整体慢 13%，转向时线速度更低、车头摆动惯量更小。
 *   实现上必须同时降 MIN 与 CURVE：MotorCommand_Calculate 会把
 *   base 夹到 [min+|c|, max-|c|]，只降 CURVE 的话平均值会被 min 顶回去。
 *   ⚠ 三个值必须同步满足 base >= curve >= sharp >= min，
 *     否则 LineFollow_ConfigIsValid() 判参数非法 → 直接进故障、电机清零。 */
#define CONTROL_BASE_SPEED_DEFAULT           (520.0f)
/* 单轮命令下限（= 内轮最低转速）。
 *
 * 原来 320：弧线上 base 400、修正 60 时，内轮只有 400-60 = 340，
 * 带负载时电机转不动 → 内轮刹住 → 车几乎走直线 → "弧线转不过去弯"。
 * 直线两轮都是 520 远高于门限，所以直线一直正常，问题只在弧线。
 *
 * 改成 450：MotorCommand 会把 base 抬到 min+|c|，于是弧线变成
 *   外轮 570 / 内轮 450 —— 两轮都在能转的区间，差速仍是 120 不变。
 * 直线（base 520 > 450）完全不受影响；直角走独立状态机也不受影响。
 * 代价：headroom 由 305 降到 240（我们差速上限本来就是 160，不受限）。 */
#define CONTROL_MIN_FORWARD_COMMAND_DEFAULT  (450.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (930.0f)
#define CONTROL_STRAIGHT_TRIM_DEFAULT        (0.0f)

#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (480.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (450.0f)
#define LINE_FOLLOW_SEARCH_SPEED_DEFAULT            (650.0f)
/* 直线/弧线的"基速分档"门限（按误差判）。贴边目标 dev=-1 时：
 *   直线稳态 dev=-1 → 误差 0   → 用 base_speed(520)
 *   弧线稳态 dev≈0  → 误差 ≥1  → 落进 curve_speed(400)
 * 原来 1.5 时弧线误差只有 1，仍然吃 520 的直线速度 → 弧线上太快，
 * 需要约 48 度/秒 的角速度和 65 的差速才能维持 → 顶不住 → 卡住。
 * 改成 0.75 后：直线(误差 0)不受影响，弧线(误差 1)自动降到 400。
 * 直角走独立状态机，完全不受这个门限影响。 */
#define LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT   (0.75f)
#define LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT      (2.5f)
#define LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT       (120.0f)
/* Brief, gentle reacquisition after a gated pause; these are driver units.
 * Defaults mix to 450/570 (or 570/450), never the normal full-strength search. */
#define LINE_FOLLOW_RESUME_SEARCH_MS                 (500U)
#define LINE_FOLLOW_RESUME_SEARCH_BASE               (510.0f)
#define LINE_FOLLOW_RESUME_SEARCH_CORRECTION         (60.0f)

#define LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT          (2500U)
#define LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT         (10U)
/* 回到"直线档"后仍保持弯道速度多少拍。原来 20 拍太短，
 * 弧线上误差在 0/1 之间跳时 520<->400 会来回切；100 拍 = 100 ms。 */
#define LINE_FOLLOW_CURVE_HOLD_CYCLES                (100U)
#define LINE_FOLLOW_BASE_STEP                       (10.0f)
/* 同方向偏差持续 30 ms，或已到外探头时进入强修正。
 * 45% 内轮下限只在强修正中启用；普通循迹仍保持 55% 下限。 */
/* 同方向偏差持续 200 ms，或已到外探头（|dev| >= 1.5）时进入"强修正"：
 * 修正量 ×1.8、内轮下限放到 450。
 * 200 ms 是区分"直线"与"弧线"的关键时间常数 ——
 * 直线上的短暂偏离在它之前就修正完了，只有弧线上那种持续偏差才触发放大。
 * 历史：3 ms（任何偏差都放大，把分级响应压回开关量）
 *        → 50 ms → 200 ms（本次，配合 Kp 降到 85）。 */
#define LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES         (20U)
/* 强修正退出条件：居中或反向持续这么多周期才撤销放大。
 * 原来是"dev 一归零就立即撤销"，而弧线上偏差会在 0 与 ±1 之间来回跳，
 * 导致放大以 2.5 倍幅度反复通断 —— 实测表现为弧线卡顿。
 * 400 ms 的释放在弧线上足够长，不会中途退出；出弯后又会自动恢复轻修正。 */
#define LINE_FOLLOW_HARD_TURN_RELEASE_CYCLES         (400U)
/* 强修正触发门限（按误差判，误差 = 偏差 - 目标）。
 * 贴边目标 dev=-1，所以：
 *   dev =  0（内侧两个灯都亮）→ 误差 +1  ← 现场要求"立刻反应"，必须触发
 *   dev = -1（目标，一个内灯）→ 误差  0  ← 不触发
 * 原来 1.5 时 dev=0 不触发，要等同向 200 拍（200 ms）才放大，表现为"反应慢"。
 * 改成 1.0 后内侧两灯一亮立刻给足差速。 */
#define LINE_FOLLOW_HARD_TURN_DEVIATION              (1.5f)
#define LINE_FOLLOW_HARD_TURN_MIN_FORWARD            (450.0f)
#define LINE_FOLLOW_HARD_TURN_GAIN                   (2.8f)

/* ===== 只压到单个外探头时，降低两轮转速差 =====
 *
 * 现象：只有左外或右外一个探头压线（|偏差| = 3）时，车会猛地甩一下。
 * 原因：偏差 3 → 误差 2 → Kp 60 × 2 = 120，再叠加强修正 ×2.5 与内轮下限 450，
 *       实测两轮差可以到 480（内轮 450 / 外轮 930），对"只是偏了一点"来说过猛。
 *
 * 对策：判定"只有外侧探头压线"时，
 *   ① 关闭强修正放大（不再 ×2.5）
 *   ② 修正量乘以 OUTER_ONLY_SCALE
 *
 * 调参：
 *   还嫌猛 → OUTER_ONLY_SCALE 0.6 → 0.4
 *   修正太慢、车回不来 → 0.6 → 0.8 */
#define LINE_FOLLOW_OUTER_ONLY_DEVIATION             (2.5f)
/* 修正量的绝对上限（不再用比例缩放，改成限幅 —— 这样左右两侧行为对称：
 * 贴边目标 -1 时，左外对应误差 -2、右外对应误差 +4，按比例缩放会让两边不一致） */
/* ===== 偏航角速度阻尼（治"弧线内扣 / 车身斜穿"）=====
 * 4 路探头只测横向偏移，测不到车头方向；现场视频里车身与胶带斜交 30~40 度，
 * 探头却仍报 dev=-1（目标值），于是车带着航向误差走 → 偏移变大才猛修 → 甩头摆动。
 * 用陀螺仪测带符号角速度做负阻尼，直接压住这种绕轴摆动。
 *
 * YAW_DAMP 是"每 (度/秒) 扣减多少差速"，先给 0.8：
 *   原地转 450 占空比约 120 度/秒 → 扣 0.8*120 ≈ 96 差速，正好压住。
 * 上车后：
 *   摆动变小、车头与胶带平行 → 好用，可再试 1.0
 *   摆得更厉害（甚至发散）  → 陀螺仪极性相反，把 SIGN 改成 (-1.0f)
 *   车变"发木"、跟不上线   → 把 YAW_DAMP 降到 0.5 */
/* ===== 自适应曲率前馈（治"弧线拖到偏差大了才猛修 → 转过头"）=====
 * 贴边目标 dev=-1 时，纯 P 在弯道上必须靠持续误差才能转弯，
 * 于是车先直走、等偏差攒大再猛修，来回反复。
 * 这里学一个【有界的弯道补偿量】补上"弯道本该有的差速"。
 *
 * FF_GAIN   每拍按误差累积的速度：误差 1 时约 35 拍(35 ms)学到 70
 * FF_LIMIT  最大补偿量（R60@520 约需 65，留两倍余量）
 * FF_LEAK   真直线上每拍泄放系数（0.98 → 约 90% 在 110 ms 内泄掉）
 * FF_ERR_MIN 误差小于这个值就认为"不需要学"
 * FF_LEAK_YAW_MAX 角速度小于这个值才认为是真直线（弯道上不泄放） */
#define LINE_FOLLOW_FF_ENABLE                        (1U)
#define LINE_FOLLOW_FF_GAIN                          (2.0f)
#define LINE_FOLLOW_FF_LIMIT                         (120.0f)
/* Slow release even while rotating; prevents a self-maintained bias. */
#define LINE_FOLLOW_FF_TURN_LEAK                    (0.998f)
#define LINE_FOLLOW_FF_LEAK                          (0.98f)
#define LINE_FOLLOW_FF_ERR_MIN                       (0.25f)
#define LINE_FOLLOW_FF_LEAK_YAW_MAX                  (10.0f)
#define LINE_FOLLOW_YAW_DAMP_ENABLE                  (1U)
#define LINE_FOLLOW_YAW_DAMP                         (0.8f)
#define LINE_FOLLOW_YAW_DAMP_SIGN                    (1.0f)
#define LINE_FOLLOW_YAW_DAMP_LIMIT                   (120.0f)
/* yaw 变化超过这个角度才更新角速度估计（太小会被量化噪声干扰） */
#define LINE_FOLLOW_YAW_RATE_MIN_DEG                 (0.3f)
/* 慢平均系数：用来分离"稳态转弯"与"摆动"。1/alpha = 时间常数(ms)。
 * 0.02 → 约 50 ms。弧线上的恒定转弯会被慢平均跟上，阻尼归零；
 * 只有快速反向的摆动才被抑制。 */
#define LINE_FOLLOW_YAW_RATE_SLOW_ALPHA              (0.02f)
#define LINE_FOLLOW_OUTER_ONLY_MAX_CORRECTION        (72.0f)
/* Direct four-sensor recovery uses a two-wheel differential at all times.
 *
 * ===== 2026-10-30 第五轮：内档差速 450 → 240 =====
 * 现象：加去抖 + 降基速后直线摆头减轻但不明显，弧线仍正常。
 * 分析：内档差速 450 对应最小转弯半径 19cm，而赛道弧线是 60cm ——
 *       控制器长期工作在"3 倍过猛"的区间，直线小幅偏差被放大成急转，就是摆头。
 *       弧线的硬下限是差速 145（半径 60cm），所以 240（半径 36cm）对弧线
 *       仍有 1.7 倍余量，这一步不会牺牲弧线。
 * 几何：R = (W/2)·(v1+v2)/(v1-v2)，W 取 15cm
 *   450 → 19cm       240 → 36cm       145 → 60cm（R60 临界）
 *
 * 若直线仍摆头：继续收到 200（半径 44cm），这已是 R60 的安全下限。
 * 若弧线开始切内：回到 280。
 * 外档保持 320/940 不动 —— 直角与急弯靠它，且直角问题尚未处理。 */
#define LINE_FOLLOW_INNER_FAST_COMMAND_DEFAULT       (640.0f)
#define LINE_FOLLOW_INNER_SLOW_COMMAND_DEFAULT       (400.0f)
#define LINE_FOLLOW_OUTER_FAST_COMMAND_DEFAULT       (940.0f)
#define LINE_FOLLOW_OUTER_SLOW_COMMAND_DEFAULT       (320.0f)
/* 四灯全灭时先「直行」多少个控制周期再开始差速搜索。
 * 80 周期 ≈ 80 ms —— 丢线后还要直冲 80 ms 才肯找线，弯道上这就是「反应慢」。
 * 收到 25 周期（≈25 ms），一丢线几乎立刻开始朝最后方向找。 */
#define LINE_FOLLOW_ALL_OFF_SLOW_CYCLES              (25U)

/* =====================================================================
 * 直角（90°）处理 —— 重写版
 *
 * 三个阶段，规则只有一条：
 *   ① 判定：线丢了并且已持续 ARM 周期 + 丢线前 |偏差| 峰值 >= PEAK
 *            （说明线是被"扫向一侧"之后消失的，直线偶发掉线没有这个过程）
 *            + 本次丢线没试过 + 不在冷却期
 *   ② 执行：停稳 SETTLE → 原地转向，方向 = 最后看到线的一侧
 *            完成条件只认【陀螺仪到角】(TARGET_DEG)；
 *            陀螺仪无数据时退化为一固定时长（TIME_PIVOT）
 *            转超 MAX_DEG 还没到角就放弃，退回前进搜索
 *   ③ 收尾：设冷却（防出弯误判）+ 出弯软启动（修正量从 RECOVER_MIN 渐升）
 *
 * 明确不做的事：
 *   ✗ 不在转向期间"看到线就退出"——探头扫过新线的窗口只有几十毫秒，必然提前退出
 *   ✗ 不做"单段超时后反向再扫"——那正是"转 30 度后反着转"的根源
 *   ✗ 不向前滚（会先掉出线外）
 * ===================================================================== */
#ifndef LINE_FOLLOW_CORNER_ENABLE
#define LINE_FOLLOW_CORNER_ENABLE                    (1)
#endif

/* 丢线持续多久才算"线真的断了"（不是一次抖动）。
 *
 * 从 100 收到 50：原来丢线后车会先在【前进搜索 650】里跑 100 ms，
 * 相当于"识别到直角还往前冲 3~4 cm"，然后才开始转 —— 现场很明显。
 * 现在判定要求三灯特征，证据已经足够，50 ms 就够滤掉抖动。
 * 再往前冲的代价：约 1.5 cm。 */
/* Brief all-black overlaps are crossings, not immediate faults. */
#ifndef LINE_FOLLOW_ALL_BLACK_CROSSING
#define LINE_FOLLOW_ALL_BLACK_CROSSING               (1U)
#endif
#define LINE_FOLLOW_ALL_BLACK_MAX_CYCLES             (150U)
#define LINE_FOLLOW_ALL_BLACK_CREEP_DUTY              (450.0f)

#define LINE_FOLLOW_CORNER_ARM_CYCLES                (50U)
/* 三灯特征的有效窗口：看到"两内 + 一侧外"之后，这么久之内丢线就算直角。
 * 400 ms 足够覆盖"特征出现 → 线断开"之间的间隔，又不会太久。 */
#define LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES         (400U)
/* 停稳分两段：
 *   ① 主动反向刹车 STOP：两轮给负命令，把冲进拐角的前进惯性真正刹住。
 *      原来只是把命令清零，车会靠惯性继续滑 1~2 cm，
 *      滑动中还可能带出偏航 —— 转到 90 度时看起来就"转过头"。
 *   ② 静止保持 SETTLE：两轮 0，等车身稳下来。 */
#define LINE_FOLLOW_CORNER_STOP_DUTY                 (300.0f)
#define LINE_FOLLOW_CORNER_STOP_CYCLES               (70U)
#define LINE_FOLLOW_CORNER_SETTLE_CYCLES             (50U)
/* 原地转向占空比 */
#define LINE_FOLLOW_CORNER_PIVOT_DUTY                (500.0f)
/* 陀螺仪可用时的目标转角与上限 */
/* Field adjustment: 85 -> 95 degrees to add about 10 degrees to the turn.
 * Pivot duty 450 -> 500 is a small speed increase; verify on the actual floor. */
#define LINE_FOLLOW_CORNER_TARGET_DEG                (95.0f)
/* 陀螺仪不可用时的转向时长（不反转，到时间就收工） */
#define LINE_FOLLOW_CORNER_TIME_PIVOT_CYCLES         (900U)
/* 绝对上限：不管陀螺仪说什么，转到这么久必须结束。
 * 防"陀螺仪中途不再更新、角度冻结"导致一直转下去。 */
#define LINE_FOLLOW_CORNER_MAX_CYCLES                (1600U)
/* 刹车脉冲：到达目标角后，反向给一小段，抵消车身的角速度惯性。
 * 原来到角直接把两轮清零，车会靠惯性继续转过头 10~20 度。 */
#define LINE_FOLLOW_CORNER_BRAKE_CYCLES              (120U)   /* 陀螺仪无效时的定长兜底 */
#define LINE_FOLLOW_CORNER_BRAKE_DUTY                (340.0f)

/* 提前量：按实测角速度预测"还会再转多少"，提前开始刹车。
 *
 * 只按"到 90° 才刹"的话，车到 90° 时仍带角速度，刹车本身还要时间才生效，
 * 必然冲过去一点（这是"角度显示 90 但仍然过转"的直接原因）。
 *
 * 判定：当前角 + 角速度 × 提前量 >= 90°  →  立即开始刹
 *   角速度 150°/s、提前量 120 ms → 多算 18° → 在 72° 就开始刹，最终落在 90° 附近。 */
#define LINE_FOLLOW_CORNER_BRAKE_LEAD_CYCLES         (120U)

/* ===== 角度收尾（保证最终落在 90°）=====
 *
 * 刹车参数是"开环"的：不管怎么调，只能对某一种进弯速度刚好
 * （实测：第一次好、后续过转；把刹车改猛之后又变成第一次也过转）。
 *
 * 所以刹车结束后再加一段【按实测最终角度】的慢速修正：
 *   最终 > 90+容差 → 反向慢转回来
 *   最终 < 90−容差 → 同向慢转补足
 * 力度只有 FIX_DUTY（远小于转向 450），不会打滑；最多 FIX_MAX_CYCLES。
 *
 * 这样直角角度就是闭环的，不再依赖电量/地面/进弯速度。 */
#define LINE_FOLLOW_CORNER_FIX_TOLERANCE_DEG         (3.0f)
#define LINE_FOLLOW_CORNER_FIX_DUTY                  (250.0f)
#define LINE_FOLLOW_CORNER_FIX_MAX_CYCLES            (300U)

/* ===== 闭环刹车（按实测角速度决定何时停）=====
 *
 * 问题：固定 120 ms 的刹车无法同时适配不同的角速度
 *   → 转得快（电足/地面滑）刹不住，转过头 20~30 度
 *   → 转得慢（电低/阻力大）刹过头，转不够 90 度
 * 这就是"大部分过转、少部分欠转"的根源。
 *
 * 做法：到 90° 后继续反向刹车，每个周期用陀螺仪 yaw 的变化率算角速度，
 *       角速度降到 BRAKE_RATE_STOP 以下（车真的不转了）就立即结束。
 *       至少刹 BRAKE_MIN_CYCLES、最多刹 BRAKE_MAX_CYCLES。
 *
 * 调参：
 *   还过转  → BRAKE_RATE_STOP 20 → 10（要求更"静"才停），或 BRAKE_DUTY 260 → 320
 *   变欠转  → BRAKE_RATE_STOP 20 → 35
 *   刹太慢（车停住后还要愣一下）→ BRAKE_MIN_CYCLES 40 → 20 */
#define LINE_FOLLOW_CORNER_BRAKE_RATE_STOP           (20.0f)  /* 度/秒 */
#define LINE_FOLLOW_CORNER_BRAKE_MIN_CYCLES          (40U)
#define LINE_FOLLOW_CORNER_BRAKE_MAX_CYCLES          (400U)
/* yaw 单次变化超过这个角度才当成一次有效采样（滤掉量化噪声） */
#define LINE_FOLLOW_CORNER_RATE_MIN_DEG              (0.05f)
/* 这么久 yaw 都没变化 → 认为角速度为 0 */
#define LINE_FOLLOW_CORNER_RATE_TIMEOUT_CYCLES       (120U)
/* 直角完成后的冷却：这段时间不再判直角（防出弯时把"内+外"图案当成新直角） */
#define LINE_FOLLOW_CORNER_COOLDOWN_CYCLES           (800U)
/* 出弯软启动：修正量在这么多周期内从 RECOVER_MIN 升到 100% */
#define LINE_FOLLOW_CORNER_RECOVER_CYCLES            (400U)
#define LINE_FOLLOW_CORNER_RECOVER_MIN               (0.4f)

/* After a completed right-angle turn, soften ONLY the two-inner pattern.
 * Driver units: 80 = 8 percentage points of wheel duty difference, not RPM.
 * Fade the cap toward the normal range over the final 500 ms. */
#define LINE_FOLLOW_POST_CORNER_GENTLE_MS            (3500U)
#define LINE_FOLLOW_POST_CORNER_DIFF_MAX             (80.0f)
#define LINE_FOLLOW_POST_CORNER_FADE_MS              (500U)


/* ===== 编码器闭环配平（治"落地后两轮负载不对称 → 跑偏 → 摆头"）=====
 *
 * 实测结论：架空时左右脉冲一致，落地后差异明显 → 差异来自负载不对称
 * （轮胎摩擦、减速箱阻力、重量分布），不是代码或电机本身。
 * 控制器只能等偏差出现后才修正，于是"被拖偏 → 修正 → 再被拖偏"，
 * 这就是直线摆头的主因。
 *
 * 处理：用编码器实测脉冲差做积分，直接加到左右轮命令上，
 *       让两轮实际转速趋于相等 —— 扰动在驱动层就被抵消，
 *       不需要等它变成横向偏差。
 * 只在左右命令相等（直线）时积分；转弯时冻结并缓慢泄放。
 *
 * ★ 现场验证（10 秒）：落地直线行驶，看屏幕底部 L / R 两个数
 *     → 逐渐趋于相等 = 符号正确，配平生效
 *     → 差距越来越大 = 符号反了，把 ENCODER_BALANCE_SIGN 改成 (-1.0f)
 *
 * 调法：
 *   收敛太慢 → KI 0.03 → 0.06
 *   收敛后仍在抖 → KI 降到 0.015，或 LIMIT 80 → 50
 *   两边机械差异特别大 → LIMIT 提到 120 */
/* 【第 1 步验证】编码器配平暂时关闭。
 * 原因：配平量被无条件加到输出上（命令为 0 时两轮仍得 ±balance），
 * 而且 DECAY=1.0 等于不泄放，积分会一路顶到 ±120 极限，
 * 表现为"左轮总比右轮快"+"丢线时仍在转"。
 * 先关掉它，区分是配平导致还是电机本身差异。 */
#define ENCODER_BALANCE_ENABLE                       (0)
#define ENCODER_BALANCE_KI                           (0.05f)
#define ENCODER_BALANCE_LIMIT                        (120.0f)
/* 转弯期间冻结（不衰减）：原来 0.98 会把学到的补偿慢慢泄掉，
 * 表现为"刚走直线差得少、过几个弯又开始差得多"。设为 1.0 即完全保持。 */
#define ENCODER_BALANCE_DECAY                        (1.0f)
#define ENCODER_BALANCE_SIGN                         (+1.0f)

/* ===== 命令斜率限制（治"修正时一顿一顿"，且不动稳态）=====
 *
 * 位置：最后一级（送进电机驱动之前）。所以它能一次覆盖所有突变来源：
 *   ① 四路数字量的状态跳变：520/520 → 460/340，差速阶跃 120
 *   ② 强修正的切入/退出：差速 120 ↔ 300
 *   ③ 直角原地转向启动：520 → +330/-330
 *
 * 与前几轮的区别：之前改的是控制律内部（Kp、差速、倍率斜坡），
 * 每次都牵动稳态值或别的场景；斜率限制只约束"变化多快"，稳态该多少还是多少。
 *
 * 取值：SLEW 6（驱动命令/毫秒，满量程 1000）
 *   120 的阶跃 ≈ 20 ms 建立完成，相对机械响应（几十~上百 ms）可忽略。
 *   太"软"、修正跟不上 → 调到 10~15
 *   还觉得顿 → 调到 3~4（但会明显拖慢修正） */
#define CONTROL_SLEW_ENABLE                          (1)
#define CONTROL_SLEW_PER_MS                          (6.0f)

/* ===== 出弯后摆头的阻尼项（治"纯 P 控制的极限环"）=====
 *
 * 背景：为了治"居中时两轮转速不一致"，Kd 曾被设为 0 —— 于是整台车没有任何阻尼。
 * 出弯时车身带残余偏航角速度，纯 P 只能等偏差建立起来才反向修正，
 * 必然过冲、再反向过冲，形成周期约 1~2 秒的长摆。
 *
 * 但不能重蹈覆辙：对 0/±1/±2/±3 的量化阶跃直接求导，等于放大量化噪声
 * （上次 dev=0 时两轮差 21% 就是这个造成的）。
 *
 * 这里改成三步：
 *   ① 先对偏差做一阶低通（ALPHA=0.15，τ≈7 ms），把阶跃变成斜坡；
 *   ② 再对滤波后的值求"每周期变化量"，不除以 dt，避免 1000 倍放大；
 *   ③ 偏差为 0 时不施加 —— 保证居中时两轮严格同速（这是硬要求）。
 *
 * 调法：
 *   摆得更厉害 / 发抖 → GAIN 250 → 150
 *   摆头压不住 → GAIN 250 → 400，或 ALPHA 0.15 → 0.25
 *   要完全关掉 → DAMP_ENABLE 0（行为回到上一版） */
#define LINE_FOLLOW_DAMP_ENABLE                      (1)
#define LINE_FOLLOW_DAMP_ALPHA                       (0.15f)
#define LINE_FOLLOW_DAMP_GAIN                        (250.0f)
#define LINE_FOLLOW_DAMP_LIMIT                       (80.0f)

/* ===== 方案 B：贴边循迹（把控制目标从居中移到偏一格）=====
 * 四路数字量只有 4 档。若两个内探头间距大于线宽，偏差 0 这个状态物理上不存在，
 * 控制器要追的目标不存在，只能在档位之间跳。把设定值移到 -1：
 * 车贴着一侧边缘稳定行驶，得到真正的稳定平台。
 *   误差 = 实际偏差 - 设定值
 *   dev = -1（目标）→ 误差 0 → 两轮严格同速
 *   dev =  0        → 误差 +1 → 轻修
 *   dev = -2        → 误差 -1 → 轻修
 * 可改：-1 贴右边缘 / +1 贴左边缘 / 0 回到居中目标。 */
#ifndef LINE_FOLLOW_TARGET_DEVIATION
#define LINE_FOLLOW_TARGET_DEVIATION                 (-1.0f)
#endif
/* ===== 方案①：四路循迹状态去抖（仅离散模式使用）=====
 * 每一位要连续这么多个控制周期（1 ms/周期）都与当前生效值不同，才允许翻转。
 * 作用：黑线卡在两个探头之间时，raw bits 毫秒级来回跳，离散状态机跟着每 1 ms
 *       改一次左右轮输出，就是肉眼看到的「高频抖动」。加了它以后，
 *       短于 6 ms 的跳变会被忽略。
 * 注意：只在 LINE_FOLLOW_DISCRETE_MODE=1 时编译；切到 PD 后这段代码不参与编译。
 *       去抖只作用于 NORMAL 分支的左右轮决策；全白（丢线）与全黑走另外的分支，
 *       检测速度不受影响，不会拖慢丢线响应。 */
#ifndef LINE_FOLLOW_BIT_DEBOUNCE_CYCLES
#define LINE_FOLLOW_BIT_DEBOUNCE_CYCLES              (6U)
#endif

/* ===== 2026-10-30 生产模式：连续 PD =====
 * 1 = 离散差速硬状态机（两档开关量，参数简单但存在极限环，直线摆头）
 * 0 = 加权偏差 PD（分级响应 + 微分阻尼，出弧线后的残余摆动有明显抑制）
 * 回退方法：改回 1 即可，离散分支的档位参数一直保留在上面。 */
#ifndef LINE_FOLLOW_DISCRETE_MODE
#define LINE_FOLLOW_DISCRETE_MODE                    (0U)
#endif

#endif /* CONTROL_CONFIG_H */
