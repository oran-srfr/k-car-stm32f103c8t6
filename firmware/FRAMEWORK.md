# 比赛主流程框架

本框架实现任务调度、状态转换、HC-SR04 超声波、TFT 人机界面和 MPU6050 串口姿态/圈计时。
视觉、蜂鸣器、循迹和停车动作仍为待实现接口。
默认无有效循迹数据、无发车事件，保持等待；初始化超声波、TFT 和 USART2，不启用电机。
不能据此宣称小车已经能循迹、识别车位或完成停车。

## 文件入口

- `Core/Src/main.c`：保留 HSI 8 MHz，非阻塞调用 `App_Task(HAL_GetTick())`。
- `Core/Src/app.c`：比赛状态机、判圈计时、识别计数、停车门控。
- `Core/Inc/app.h`：输入、状态、命令类型。
- `Core/Src/app_modules.c`：模块整合入口，接入超声波、HMI 和姿态数据；视觉、循迹、停车仍是占位。
- `Core/Src/ultrasonic.c`、`ultrasonic_stm32.c`：非阻塞测距逻辑、STM32 HAL GPIO 与 TIM4/EXTI 硬件适配，见 [ULTRASONIC.md](ULTRASONIC.md)。
- `Core/Src/Attitude.c`、`MpuLink.c`、`LapTimer.c`：USART2 姿态链路、串口帧解析与陀螺仪圈计时，见 [HMI_ATTITUDE.md](HMI_ATTITUDE.md)。
- `Core/Src/HCI.c`、`lcd.c`、`lcd_init.c`、`zk.c`：ST7735 显示、ASCII/GB2312 字库和五项比赛数据界面。
- `Core/Inc/board_pins.h`：已确认接线定义，不产生硬件操作。

## 三种独立比赛模式

HMI 的显示端已接入；模式和一次发车按键边沿仍待输入层实现，并由 `Modules_Poll()` 提交。发车要求循迹有效、
超声波数据有效且不超过 200 ms、前方距离大于 80 mm。

1. `APP_MODE_LAPS`：从 A 点发车，按 B→C→D→A 顺序完成第一圈，重复后在 A 点结束。
   每圈单独计时，计时包含中途停车等待。B/C 事件触发蜂鸣接口。
2. `APP_MODE_SIDE_PARK`：循迹寻找侧方车位，收到有效车位事件进入侧方停车；
   脱离循迹线时蜂鸣，停车模块确认完成后进入结束状态。
3. `APP_MODE_REVERSE_PARK`：同上，但调用倒车入库动作接口。

停车完成后不自动驶出或恢复循迹；需新的发车操作。每次发车清零本次两圈时间和三项识别计数。
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
- `Modules_Drive()` 接收 STOP/FOLLOW/SIDE/REVERSE。接口必须非阻塞，STOP 可反复调用，
  并清除停车动作进度；初始化和结束均保持停止。未来紧急故障也应在驱动层立即关断输出。
- 前方距离 ≤80 mm 停车并蜂鸣；距离 >100 mm 才解除障碍，防阈值抖动。
  同一障碍保持期间只计数一次。丢回波或超时不代表前方安全。
- 循迹途中障碍消失且数据有效后自动恢复；循迹信号失效进入故障等待。
  停车动作中失去测距/出现障碍、停车模块失败或超过 30 秒，停止并进入故障，需重新发车。
  30 秒是待实车调整的看门狗；单个前向超声波不构成倒车方向的安全检测。
- STOP 按键优先于其他事件。所有状态机和显示在主循环执行，中断只捕获脉冲/字节。
- MPU6050 的 USART2 中断只推入 128 字节环形缓冲并重新武装接收；帧解析、Yaw 换算和圈计时在主循环执行。
- `Modules_RunStarted()` 只在线圈模式启动陀螺仪计时；停止、故障和完成状态都会停表。侧方/倒库模式不启动圈计时。

## 已确认接线与待实现项

| 信号 | 引脚 |
|---|---|
| 右电机 AIN1 / AIN2 | PB6 / PB5 |
| 左电机 BIN1 / BIN2 | PB7 / PB8 |
| 左 PWMB / 右 PWMA | PA6 TIM3_CH1 / PA7 TIM3_CH2 |
| 超声波 Trig / Echo | PA4 / PA5 |
| MPU6050 模块 TX / RX | PA3 / PA2，USART2 |
| 无源蜂鸣器 | PA0 TIM2_CH1，经三极管驱动 |
| TFT SCL / MOSI / CS / DC / MISO | PB14 / PB15 / PA1 / PA12 / PB13 |

引脚已确认不等于电机正反转电平已标定。电机 PWM、PID、循迹黑白电平、位置识别、
视觉通信、蜂鸣器及停车轨迹仍待实现；超声波、显示和姿态解析已接入，尚待实测。当前仍使用 HSI 8 MHz，
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

`tests/run_tests.ps1` 还可运行 `test_laptimer` 与 `test_mpulink`，覆盖慢速累计、
±180° 解卷绕、坏帧/断流、串口重同步、校验和、环形缓冲和 Yaw 队列。
当前机器没有主机 gcc，因此新增测试源码已交叉编译检查，但尚未在主机执行。
状态机测试覆盖两圈顺序与 A 点结束、障碍停走和去重、无效/过期输入、停止优先级、
两种停车模式及失败/超时、系统毫秒计数回绕和显示节流。尚未验证实车行为。
