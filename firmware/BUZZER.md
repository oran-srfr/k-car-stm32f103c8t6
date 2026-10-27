# 无源蜂鸣器模块

## 硬件与资源

使用 PA0 / TIM2_CH1，当前安装的是**低电平触发**蜂鸣器模块。
静音时 PA0 保持高电平，播放时通过 TIM2_CH1 的 `CC1P` 反相输出约 50% PWM。
初始化失败时 PA0 也保持高电平，避免模块持续鸣叫。若以后换成高电平触发模块，
把 `Core/Inc/buzzer_port.h` 中的 `BUZZER_ACTIVE_LOW` 改为 `0U`。
上电不自动鸣叫，不需要为每个 PWM 周期运行中断。

TIM2 由此模块独占，不改动电机的 TIM3、超声波的 TIM4 或 SWD 引脚。
硬件层根据 APB1 时钟计算 1 MHz 计数基准，并处理 APB1 分频时定时器倍频。
当前 HSI 8 MHz 时预分频为 7；TIM2 时钟为 72 MHz 时为 71。
若不能精确产生 1 MHz，或 TIM2 已重映射离开 PA0，初始化失败并保持 PA0 高电平静音。
GPIO 使用 HAL，TIM2 使用 CMSIS 寄存器，不额外启用 HAL TIM 库。

## 事件提示音

音调与时长是软件默认值，不是赛题规定的编码；可在 `Core/Src/buzzer.c` 修改。

| 事件 | 默认模式 | 主框架入口 |
|---|---|---|
| B 点 | 2400 Hz，120 ms 一声 | `APP_SOUND_B` |
| C 点 | 1800 Hz，两声各 100 ms，中间间隔 80 ms | `APP_SOUND_C` |
| 障碍停车 | 3000 Hz，三声各 120 ms，间隔 80 ms | `APP_SOUND_OBSTACLE` |
| 侧方/倒库离开循迹线 | 1200 Hz，250 ms 一声 | `APP_SOUND_LEAVE_LINE` |

`Modules_Init()` 初始化蜂鸣器；`Modules_Poll()` 每次调用 `Buzzer_Task(now_ms)`；
`Modules_Sound()` 已将上述四个比赛事件映射到提示音。两种停车任务共用离线提示音，
但主框架分别在各自任务中触发。

正常提示按 8 项 FIFO 排队，满时 `Buzzer_Play()` 返回 false，保留已排队提示。
障碍报警立即打断普通提示，并丢弃此前未播放的普通队列，避免迟来的普通声音掩盖报警。
同一次障碍报警播放中再次请求不会重复排队或从头播放；其后新增的普通提示等待报警结束。
主流程仍负责对物理事件去重，不应每次循环都调用播放请求。

时长由毫秒计数判断，支持 `HAL_GetTick()` 回绕，不使用 `HAL_Delay()`。
实际音段与静音间隔可能因主循环/显示阻塞而略微延长；延迟轮询不会省略音段或静音间隔。
模块不在中断里调用，`Buzzer_Stop()` 可立即停止并清空队列。
普通任务结束不会自动取消最后一声提示；若业务要求急停立即静音，可显式调用停止接口。

## 独立试音

在初始化后的主循环上下文中，仅调用一次：

```c
Buzzer_Play(BUZZER_POINT_B, HAL_GetTick());
```

保持现有主循环反复执行，播放会自动结束。不能把该请求不加边沿控制地放入无限循环，
否则会重复排队。测试其他提示时分别使用 `BUZZER_POINT_C`、`BUZZER_OBSTACLE`、
`BUZZER_LEAVE_LINE`。`Buzzer_IsReady()` 与 `Buzzer_IsBusy()` 可用于调试。

当前已有稳定黑线自动发车（见 [AUTO_START.md](AUTO_START.md)）；B/C 位置与停车事件仍待识别层接入，上电静音正常。

## 验证

在 `firmware/` 执行：

```sh
mkdir -p build/host
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address -g \
  -ICore/Inc Core/Src/buzzer.c tests/test_buzzer.c -o build/host/test_buzzer
build/host/test_buzzer
```

Windows 的 `tests/run_tests.ps1` 也已登记此测试。
覆盖音段/间隔、普通队列顺序与满队列、障碍抢占与重复合并、主动停止、初始化失败、
非法事件、毫秒计数回绕和延迟轮询。主机测试模拟硬件输出，不证明实物声音或电平正确。

Debug/Release 固件构建验证 STM32 硬件层可以编译链接。未连接板卡，未执行试音；
实测时确认 PA0 约 50% 反相 PWM、频率符合表格、静音时保持高，以及超声波 TIM4 仍正常工作。
