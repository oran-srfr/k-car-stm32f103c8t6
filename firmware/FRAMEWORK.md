# 比赛主流程框架

本框架实现任务调度、状态转换、HC-SR04 超声波、四路循迹/PD/TB6612 电机控制、
TFT 人机界面、MPU6050 串口姿态/圈计时和蜂鸣器。视觉和停车动作仍为待实现接口。
正常黑线连续满足 200 ms 后自动发车（见 [AUTO_START.md](AUTO_START.md)）；普通循迹不依赖超声波。初始化循迹、电机、TFT、USART2 和蜂鸣器，
但电机门控在 `APP_DRIVE_FOLLOW` 之前保持关闭。
循迹和电机参数尚未实车标定，不能据此宣称小车已完成整场任务。

## 文件入口

- `Core/Src/main.c`：保留 HSI 8 MHz，非阻塞调用 `App_Task(HAL_GetTick())`。
- `Core/Src/app.c`：比赛状态机、判圈计时、识别计数、停车门控。
- `Core/Inc/app.h`：输入、状态、命令类型。
- `Core/Src/app_modules.c`：模块整合入口，接入超声波、循迹/电机、HMI 和姿态数据；视觉、位置事件和停车仍是占位。
- `Core/Src/ultrasonic.c`、`ultrasonic_stm32.c`：非阻塞测距逻辑、STM32 HAL GPIO 与 TIM4/EXTI 硬件适配，见 [ULTRASONIC.md](ULTRASONIC.md)。
- `Core/Src/Attitude.c`、`MpuLink.c`、`LapTimer.c`：USART2 姿态链路、串口帧解析与陀螺仪圈计时，见 [HMI_ATTITUDE.md](HMI_ATTITUDE.md)。
- `Core/Src/HCI.c`、`lcd.c`、`lcd_init.c`、`zk.c`：ST7735 显示、ASCII/GB2312 字库和五项比赛数据界面。
- `Core/Src/buzzer.c`、`buzzer_stm32.c`：非阻塞提示音调度与 TIM2 PWM，见 [BUZZER.md](BUZZER.md)。
- `Core/Src/line_sensor.c`、`line_track.c`、`pd_controller.c`、`line_control.c`、`line_follow.c`、`motor_command.c`、`vehicle_control.c`、`motor_driver.c`：10 ms 循迹状态机、PD 控制、安全门控与 TIM3 电机输出，见 [LINE_FOLLOW.md](LINE_FOLLOW.md)。
- `Core/Inc/board_pins.h`：已确认接线定义，不产生硬件操作。

## 三种独立比赛模式

HMI 显示端已接入，默认两圈循迹模式，无需按键。正常黑线连续满足 200 ms 后发车；全白、全黑和搜索状态不会触发起跑。超声波不参与普通循迹的发车或运行控制。

1. `APP_MODE_LAPS`：从 A 点发车，按 B→C→D→A 顺序完成第一圈，重复后在 A 点结束。
   每圈单独计时，计时包含中途停车等待。B/C 事件触发蜂鸣接口。
2. `APP_MODE_SIDE_PARK`：循迹寻找侧方车位，收到有效车位事件进入侧方停车；
   脱离循迹线时蜂鸣，停车模块确认完成后进入结束状态。
3. `APP_MODE_REVERSE_PARK`：同上，但调用倒车入库动作接口。

停车完成后不自动驶出或恢复循迹；需显式重新布防，再满足起跑条件。每次发车清零本次两圈时间和三项识别计数。
侧方/倒库计数在识别目标并接受停车任务时增加，每次任务只加一次，不冒充停车成功次数。
五项显示数据由 `Modules_Display()` 每 100 ms 更新：两圈耗时来自陀螺仪 `LapTimer`，
三个识别计数来自 `AppStatus`。应用层仍用几何 A/B/C/D 事件判定圈数，不用累计 720° 冒充到达 A 点。

## 接口约定

- `Modules_Poll()` 每次完整填写输入快照。按键、位置、车位和离线事件必须由模块消抖、去重，
  一次事件只上报一次；未收到新事件时填空事件。停车结果默认 `APP_PARK_BUSY`。
- **A/B/C/D 是场地几何位置，没有印刷标记。** 位置模块后续应结合姿态、编码器里程、弯直道变化
  与起点基准生成事件；这里不实现识别，也不以累计转角 720°直接认定停到 A 点。
- K230 只提供识别信息；`APP_PARK_DONE` 应由 STM32 停车控制器确认已停车、方向正确且车身入位，
  不能仅凭收到视觉串口消息宣告完成。具体协议解析仍待实现。
- `Modules_Drive()` 接收 STOP/FOLLOW/SIDE/REVERSE。只有 FOLLOW 打开循迹电机门控；
  其它命令立即关闭门控并清零 PWM/方向。接口非阻塞，STOP 可反复调用。
- 前方距离 ≤80 mm 停车并蜂鸣；距离 >100 mm 才解除障碍，防阈值抖动。
  同一障碍保持期间只计数一次。丢回波或超时不代表前方安全。
- 循迹途中障碍消失且数据有效后自动恢复；循迹信号失效进入故障等待。
  停车动作中失去测距/出现障碍、停车模块失败或超过 30 秒，停止并进入故障，需重新发车。
  30 秒是待实车调整的看门狗；单个前向超声波不构成倒车方向的安全检测。
- STOP 按键优先于其他事件。所有状态机和显示在主循环执行，中断只捕获脉冲/字节。
- MPU6050 的 USART2 中断只推入 128 字节环形缓冲并重新武装接收；帧解析、Yaw 换算和圈计时在主循环执行。
- `Modules_RunStarted()` 只在线圈模式启动陀螺仪计时；停止、故障和完成状态都会停表。侧方/倒库模式不启动圈计时。
- 循迹每 10 ms 采样 PB0/PB1/PB10/PB11，默认黑线为低电平。PD 输出只在正常循迹状态生成；
  丢线先搜索，超时、全黑或参数错误进入故障并关闭电机。电机 PWM 使用 TIM3 CH1/CH2（PA6/PA7）。

## 已确认接线与待实现项

| 信号 | 引脚 |
|---|---|
| 右电机 AIN1 / AIN2 | PB6 / PB5 |
| 左电机 BIN1 / BIN2 | PB7 / PB8 |
| 左 PWMB / 右 PWMA | PA6 TIM3_CH1 / PA7 TIM3_CH2 |
| 超声波 Trig / Echo | PA4 / PA5 |
| 四路循迹 X1～X4 | PB0 / PB1 / PB10 / PB11 |
| MPU6050 模块 TX / RX | PA3 / PA2，USART2 |
| 无源蜂鸣器 | PA0 TIM2_CH1，经三极管驱动 |
| TFT SCL / MOSI / CS / DC / MISO | PB14 / PB15 / PA1 / PA12 / PB13 |

引脚已确认不等于电机正反转电平和循迹极性已标定。位置识别、视觉通信及停车轨迹仍待实现；
蜂鸣器、超声波、循迹、电机、显示和姿态解析已接入，尚待实测。当前仍使用 HSI 8 MHz，
定时器预分频必须按实际时钟计算，不能照抄 72 MHz 下的 PSC=71。

## 验证

在 `firmware/` 执行正常 Debug/Release CMake 预设构建，不需要强制包含 HAL 的临时参数。
主机状态机测试使用模拟输入与输出，不驱动硬件：

```sh
mkdir -p build/host
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address -g \
  -ICore/Inc Core/Src/app.c tests/test_app.c -o build/host/test_app
build/host/test_app
```

`tests/run_tests.ps1` 还可运行 `test_laptimer`、`test_mpulink` 与 `test_tracking`，
覆盖慢速累计、±180° 解卷绕、坏帧/断流、串口重同步，以及循迹几何、PD 和电机混合。
自动发车的主机测试与验证方法见 [AUTO_START.md](AUTO_START.md)。
状态机测试覆盖两圈顺序与 A 点结束、障碍停走和去重、无效/过期输入、停止优先级、
两种停车模式及失败/超时、系统毫秒计数回绕和显示节流。尚未验证实车行为。
