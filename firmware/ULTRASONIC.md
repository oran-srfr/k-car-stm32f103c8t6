# HC-SR04 测距模块

## 接线与资源

| HC-SR04 | 连接 |
|---|---|
| Trig | PA4 |
| Echo | 分压到约 3.3 V 后接 PA5 |
| VCC | 模块额定 5 V 电源 |
| GND | 与 STM32 共地 |

Echo 通常输出 5 V，PA5 不可直接接入。可用 Echo→1 kΩ→PA5→2 kΩ→GND 分压。
本驱动针对有 Trig/Echo 接口的 HC-SR04，不适用于需要 MCU 直接输出 40 kHz 激励的裸探头。

独占 TIM4，1 MHz、16 位自由计数；使用内部 CH1 比较中断结束触发脉冲，
不输出 TIM4_CH1 到 PB6，因此不会改动电机方向引脚。Echo 通过 EXTI5 双沿捕获，
在 `EXTI9_5_IRQHandler` 中分发；未来启用 EXTI6～9 时需扩展同一中断，不能重复定义。
TIM2/TIM3 留给蜂鸣器和电机，PA13/PA14 的 SWD 不变。

TIM4 的预分频由 APB1 实际时钟计算，正确处理 APB1 分频后的定时器时钟倍频：
目前 HSI 8 MHz 时 PSC=7；若后续时钟树给 TIM4 提供 72 MHz，则 PSC=71。
不支持精确分频到 1 MHz 的时钟配置会返回 `ULR_HW_ERROR`。
GPIO 使用 HAL，专用 TIM4 使用 CMSIS 寄存器配置，无需启用 HAL TIM 模块或额外添加 TIM 驱动。

## 使用

`Modules_Init()` 已调用 `Ulr_Init(HAL_GetTick())`，主循环中 `Sonar_Poll()`
反复调用 `Ulr_Task(now_ms)`，然后读取 `Ulr_GetReading()`。
不需要在主函数添加阻塞延时，也不要从 SysTick 中调用测距任务。

- 最短发射周期 60 ms。PA4 拉高后，TIM4 比较中断在约 10 µs 后拉低；
  中断延迟可使高电平稍长，实际波形仍需示波器/逻辑分析仪验证。
- Echo 上升沿记录计数，下降沿记录高电平宽度；中断只保存原始数据。
- 主循环计算 `距离(mm) = 脉宽(µs) × 343 / 2000`，标称声速对应约 20 ℃。
  未实现温度补偿或中值滤波，以免短距离障碍被历史值延迟。
- 距离有效范围 20～4000 mm；16 位计数差支持一次回绕，Echo 超时为 30 ms。
  同时使用毫秒截止时间拒绝迟到回波，避免超时后计数器多圈回绕造成假距离。
- 超短脉冲按“过近”处理并停止，不据此放行；可能的毛刺误停需在实车测量后调整。
- 完整样本带下降沿时间戳；等待下一次测量时保留上次样本，超时后立即标记无效。
  应用层再检查 200 ms 新鲜度。

| 状态 | 主框架处理 |
|---|---|
| `ULR_OK` | 有效距离，≤80 mm 停车，>100 mm 解除障碍 |
| `ULR_TOO_CLOSE` | 作为有效的 0 mm 距离处理，立即触发停车 |
| `ULR_NOT_READY` / `ULR_NO_ECHO` | 无效，等待/停车 |
| `ULR_TOO_FAR` / `ULR_ECHO_STUCK` / `ULR_HW_ERROR` | 无效，等待/停车 |

无回波不等于前方安全，因此空旷场地没有反射目标时也可能保持停车。
如需改变这一策略，应先结合实际传感器覆盖范围设计，而不是把超时伪造为远距离。
前向超声波不代表倒车方向具有障碍检测能力。

## 验证与实机调试

在 `firmware/` 下执行主机测试：

```sh
mkdir -p build/host
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address -g \
  -ICore/Inc Core/Src/ultrasonic.c tests/test_ultrasonic.c -o build/host/test_ultrasonic
build/host/test_ultrasonic
```

测试覆盖正常脉宽换算、80 mm 阈值、60 ms 周期、无回波/缺下降沿、触发中断缺失、
超时后恢复、近/远越界、Echo 卡高、初始化失败、迟到/重复边沿，以及微秒/毫秒回绕。
硬件适配文件参与 Debug/Release 固件构建；主机测试模拟硬件接口，不证明实际电平或测距精度。

实机应先保持电机断开，通过 SWD 在 `Ulr_Task()` 返回后查看 `reading` 或 `sample`：
确认 PA4 脉冲、PA5 电平和 Echo 脉宽，再用 8 cm、10 cm、50 cm、100 cm 平面目标验证。
检查遮挡、断开 Echo、无目标时无效状态与停车逻辑。当前未连接板卡，未执行这些实测。

当前超声波不参与普通循迹模式的发车或运行控制，只在侧方停车、倒车入库模式中
参与安全检查，详见 [AUTO_START.md](AUTO_START.md)。
