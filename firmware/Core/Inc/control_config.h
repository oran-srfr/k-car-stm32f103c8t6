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
#define CONTROL_MIN_FORWARD_COMMAND_DEFAULT  (320.0f)
#define CONTROL_MAX_FORWARD_COMMAND_DEFAULT  (930.0f)
#define CONTROL_STRAIGHT_TRIM_DEFAULT        (0.0f)

#define LINE_FOLLOW_CURVE_SPEED_DEFAULT             (400.0f)
#define LINE_FOLLOW_SHARP_CURVE_SPEED_DEFAULT       (360.0f)
#define LINE_FOLLOW_SEARCH_SPEED_DEFAULT            (650.0f)
#define LINE_FOLLOW_STRAIGHT_MAX_DEVIATION_DEFAULT   (1.5f)
#define LINE_FOLLOW_CURVE_MAX_DEVIATION_DEFAULT      (2.5f)
#define LINE_FOLLOW_SEARCH_CORRECTION_DEFAULT       (120.0f)
#define LINE_FOLLOW_MAX_LOST_CYCLES_DEFAULT          (2500U)
#define LINE_FOLLOW_REACQUIRE_CYCLES_DEFAULT         (10U)
#define LINE_FOLLOW_CURVE_HOLD_CYCLES                (20U)
#define LINE_FOLLOW_BASE_STEP                       (10.0f)
/* 同方向偏差持续 30 ms，或已到外探头时进入强修正。
 * 45% 内轮下限只在强修正中启用；普通循迹仍保持 55% 下限。 */
/* 同方向偏差持续 200 ms，或已到外探头（|dev| >= 1.5）时进入"强修正"：
 * 修正量 ×1.8、内轮下限放到 450。
 * 200 ms 是区分"直线"与"弧线"的关键时间常数 ——
 * 直线上的短暂偏离在它之前就修正完了，只有弧线上那种持续偏差才触发放大。
 * 历史：3 ms（任何偏差都放大，把分级响应压回开关量）
 *        → 50 ms → 200 ms（本次，配合 Kp 降到 85）。 */
#define LINE_FOLLOW_HARD_TURN_TRIGGER_CYCLES         (200U)
/* 强修正退出条件：居中或反向持续这么多周期才撤销放大。
 * 原来是"dev 一归零就立即撤销"，而弧线上偏差会在 0 与 ±1 之间来回跳，
 * 导致放大以 2.5 倍幅度反复通断 —— 实测表现为弧线卡顿。
 * 400 ms 的释放在弧线上足够长，不会中途退出；出弯后又会自动恢复轻修正。 */
#define LINE_FOLLOW_HARD_TURN_RELEASE_CYCLES         (400U)
#define LINE_FOLLOW_HARD_TURN_DEVIATION              (1.5f)
#define LINE_FOLLOW_HARD_TURN_MIN_FORWARD            (450.0f)
#define LINE_FOLLOW_HARD_TURN_GAIN                   (2.5f)
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

/* ===== 直角（90°）处理 —— 已退回 v1 =====
 * v2（扫描式：丢线 150 ms 后原地扫描、300 ms 反向）实测导致直线出现摆头，
 * 已撤回。v1 的行为：丢线前压到外探头（|dev| >= 2）才原地转向 900 ms。
 * 直角本身仍未解决，但优先保证直线与弧线的稳定（见 调参记录.md）。 */
#ifndef LINE_FOLLOW_CORNER_ENABLE
#define LINE_FOLLOW_CORNER_ENABLE                    (1)
#endif
/* 触发：丢线前的偏差已经偏向一侧（贴边循迹下正常是 -1，拐角时会被扫到 -2/-3）。
 * 1.5 比原来的 2.0 略松，因为贴边目标本身占了一格。 */
#define LINE_FOLLOW_CORNER_TRIGGER_DEVIATION         (1.5f)
/* ★ 直角特征（现场确认）：拐弯处线同时压住【两个内探头 + 一侧外探头】。
 *   看到这个特征后 SIG_WINDOW 周期内丢线 → 判定为直角，方向由"哪一侧外探头亮"决定。
 *   原来还有一条"丢线 40 ms 就算直角"的宽泛兜底，会误伤直线与弧线，已删除。 */
/* 直角判据 = 【三灯特征出现过】AND【丢线持续 >= ARM 周期】。
 *
 * 为什么用"与"而不是单条：
 *   只看三灯特征 → 弧线上偶尔也会出现，会误触发；
 *   只看丢线时长 → 直线抖动/弧线短暂丢线也会满足；
 *   两者同时满足 → 弧线（线连续、不会持续丢线）和直线（没有三灯特征）都不会触发，
 *                  而直角（特征 + 线真的断了）一定触发。
 *
 * 窗口放宽到 400：直角处图案会在"三灯 / 两内"之间来回跳，
 * 若把窗口设得太小（曾用 80），最后一次看到三灯到真正全白超过窗口就漏判，
 * 表现为"识别不到直角、直接开过去"。 */
#define LINE_FOLLOW_CORNER_SIG_WINDOW_CYCLES         (400U)
/* 丢线持续这么久，才认为"线真的断了"而不是一次抖动。 */
#define LINE_FOLLOW_CORNER_ARM_CYCLES                (100U)
/* ① 先停稳再转：消除冲进拐角的惯性（允许停车，见需求 B1）。 */
#define LINE_FOLLOW_CORNER_SETTLE_CYCLES             (50U)
/* ② 原地转向占空比。原来 330 偏弱：前驱+后万向轮的底盘在转向时要横向拖动
 *    万向轮，330 经常转不动（视频里"斜着停住一秒"就是它）。提高到 450。 */
#define LINE_FOLLOW_CORNER_PIVOT_DUTY                (450.0f)
/* ③ 确认：探头连续这么多周期看到线才算真的找到，避免"扫到一下又丢"的来回抽搐。 */
/* 起转后的"闭眼"时间：这段时间内不承认任何"看见线"。
 * 原因：直角触发时探头往往还压在旧线附近，若一起转就认线，25 ms 后就会退出，
 * 结果车只转了 5~10 度 —— 实测的"直角转不彻底"就是这个。
 * 300 ms 在占空比 450 下大约对应 45~60 度，之后再找新线才不会提前退出。 */
#define LINE_FOLLOW_CORNER_MIN_PIVOT_CYCLES          (300U)
/* 退出直角时要求的"目标图案"容差：
 * 只接受 误差 ∈ [-1, +1]，即 偏差 ∈ {TARGET-1, TARGET, TARGET+1}。
 * 目标 TARGET = -1 → 接受 {-2, -1, 0}（线在左侧或正中），
 * 拒绝右侧图案（说明还没转到 90 度）。 */
/* ===== 陀螺仪辅助直角 =====
 *
 * 问题：原地转向的角度取决于电量、地面摩擦、万向轮阻力，同一段时间转过的角度
 *       差别很大 —— 实测"有时候能转过去，有时候转不过去"。
 * 做法：用 MPU6050 的 yaw 角测量【实际转过的角度】：
 *       起转时记下 yaw_start，转向中算 |normalize(yaw_now - yaw_start)|，
 *       转到 BLIND_DEG 才允许"看线退出"（替代原来按时间的闭眼期），
 *       转到 MAX_DEG 还没找到线就放弃转向、退回前进搜索。
 * 兜底：若陀螺仪没有数据（live=0），自动退回按时间的方式，不影响使用。
 *
 * 调法：
 *   转不够 → BLIND_DEG 55 → 65
 *   转过头 → BLIND_DEG 55 → 45，或 MAX_DEG 110 → 100
 *   陀螺仪数据不可用时看 MIN_PIVOT/LEG 两个时间参数 */
/* ===== 直角：先向前滚一段，再原地转（几何修正）=====
 *
 * 探头阵列在驱动轴前方约 8 cm，拐角丢线时探头刚越过拐角约 2 cm。
 * 此时直接原地转 90°，探头离新线的横向距离约 8-2 = 6 cm，
 * 而阵列总宽只有约 4 cm —— 够不到新线，表现为"转满了反而找不到线"。
 *
 * 所以丢线后先【向前滚一段】把这段偏移补掉（滚 5 cm 左右），
 * 再原地转 90°，探头就能正好压在新线上。
 * 占空比 450 时速度约 0.15~0.2 m/s，300 ms ≈ 4.5~6 cm。 */
/* 向前滚：默认关闭（0）。
 * 我原来按"探头在驱动轴前 8 cm"推算，认为必须滚 5 cm 才能让探头够到新线；
 * 但实测"不滚反而更容易接上"，说明实际偏移比推算值小。
 * 而且这一滚是往"线已经拐走"的区域开，会先掉出线外。
 * 需要时可以重新打开（例如探头离轴确实远的情况）。 */
#define LINE_FOLLOW_CORNER_CREEP_CYCLES              (0U)
#define LINE_FOLLOW_CORNER_CREEP_DUTY                (450.0f)
/* 陀螺仪可用时的目标转角：转到这个角度就结束转向（不看线，直接交给循迹）。
 * 比 90 略小一点，留一点余量给循迹收尾。 */
/* 目标转角：90 度（按实际测试要求）。
 *
 * 说明：转满 90 度后车头与新线平行，探头可能正好够不到新线，此时会走正常的
 * 前进搜索去重新捕获（有 MAG_LOST 预算兜底，不会判故障）。
 * 若实测"转完总是接不上"，把该值降到 84~86 即可，其它不用动。 */
#define LINE_FOLLOW_CORNER_TARGET_DEG                (90.0f)
#define LINE_FOLLOW_CORNER_BLIND_DEG                 (55.0f)
#define LINE_FOLLOW_CORNER_MAX_DEG                   (110.0f)
#define LINE_FOLLOW_CORNER_EXIT_TOLERANCE            (1.0f)
#define LINE_FOLLOW_CORNER_CONFIRM_CYCLES            (25U)
/* ④ 单方向最多转这么久，还看不到线就反向再扫一次（不倒车，只换转向方向）。 */
#define LINE_FOLLOW_CORNER_LEG_CYCLES                (700U)
/* ⑤ 总时长上限：停稳 50 + 两段 700×2 = 1450，给到 1800 留余量，
 *    超过就退回前进搜索（不判故障）。 */
#define LINE_FOLLOW_CORNER_PIVOT_TOTAL_CYCLES        (1800U)
#define LINE_FOLLOW_ALL_BLACK_CROSSING               (1)
#define LINE_FOLLOW_ALL_BLACK_CREEP_DUTY             (450.0f)
#define LINE_FOLLOW_ALL_BLACK_MAX_CYCLES             (300U)

/* ===== 出直角后的修正量软启动（治"直角之后直线摆头极其严重"）=====
 * 现象：直角原地转向完成、刚重新看见线时，偏差往往直接落在 2~3 格（外探头），
 *       而 |dev| >= 1.5 会立即触发强修正（×1.8 → 差速 480），
 *       车被猛地甩出去，再反向满力拉回，形成剧烈来回摆。
 * 处理：从 CORNER 状态退出后，修正量按 RECOVER_MIN → 100% 线性爬升，
 *       期间不叠加 1.8 倍强修正放大。
 * 调法：还甩尾 → RECOVER_CYCLES 250 → 400，或 RECOVER_MIN 0.35 → 0.25
 *       恢复太慢、直角后跟不上 → RECOVER_CYCLES 250 → 150 */
#define LINE_FOLLOW_CORNER_RECOVER_CYCLES            (250U)
#define LINE_FOLLOW_CORNER_RECOVER_MIN               (0.35f)

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
#define ENCODER_BALANCE_ENABLE                       (1)
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
