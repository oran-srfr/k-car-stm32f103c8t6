# 校电赛智能小车：Agent 快速上手

> 最后更新：2026-10-27
>
> 当前硬件目标已从 STM32F103VET6 切换为 **STM32F103C8T6**。本文是项目内的软件与接线基线；开始改代码前先读完“未确认事项”，不要根据猜测补全引脚。

## 0. Agent 必读：10 分钟上手

1. 优先打开 `k-car.code-workspace`。它包含 `firmware/` 和项目文档两个目录；编译器、调试器和构建脚本都以 `firmware/` 为工作目录。
2. 先运行 `STM32: Check tools`，再运行 `Ctrl+Shift+B`。不要先执行 `Flash`，它会真实写入板卡。
3. 引脚事实的优先级是：`firmware/Core/Inc/board_pins.h` 和实际驱动代码 > 本文的“当前代码基线” > 用户提供的引脚表。遇到冲突时先停下来核对实物，不要“顺手改成看起来更合理”的引脚。
4. 当前代码已经接好 TB6612、四路循迹、MPU6050 串口模块、HC-SR04、ST7735、蜂鸣器、编码器 A 相和 K230 串口占位；视觉解析、A/B/C/D 位置事件、实体停车/模式按键和自动泊车动作仍未实现。
5. 不要给 C8T6 套用 F407/F103VE 的 512 KiB Flash、64 KiB RAM、启动文件或 CubeMX 工程。当前器件宏是 `STM32F103xB`，链接布局是 64 KiB Flash + 20 KiB RAM。
6. 修改引脚、外设或 HAL 模块时，至少同步检查 `board_pins.h`、对应驱动、`firmware/CMakeLists.txt`、`firmware/Core/Inc/stm32f1xx_hal_conf.h` 和本文，避免代码、构建和接线文档三套漂移。

## 1. 项目概览

- 主控：STM32F103C8T6，Cortex-M3，64 KiB Flash，20 KiB SRAM。
- 调试/烧录：ST-Link，SWD，SWDIO=PA13，SWCLK=PA14。
- 固件环境：STM32CubeF1 HAL/CMSIS、GNU Arm GCC、CMake、Ninja。
- IDE：VS Code，STM32 官方扩展、Microsoft C/C++、CMake Tools；工作区文件为 `k-car.code-workspace`，同时显示固件目录和项目文档目录。
- 固件状态：已加入比赛状态机、HC-SR04 测距、ST7735 人机界面、MPU6050 姿态/圈计时、蜂鸣器和四路循迹/TB6612 电机控制栈；循迹参数尚未实车标定，视觉和停车动作仍未实现。流程见 [firmware/FRAMEWORK.md](firmware/FRAMEWORK.md)，循迹说明见 [firmware/LINE_FOLLOW.md](firmware/LINE_FOLLOW.md)。
- 非固件资料：采购清单在 `output/pdf/`，生成脚本为 `build_k_bom_pdf.py`，图片与中间产物在 `_k_figs/`、`tmp/`。

目标芯片配置已经落实在工程中：

| 项目 | 当前值 |
|---|---|
| 器件宏 | `STM32F103xB` |
| CMSIS 设备头 | `stm32f103xb.h` |
| 启动文件 | `startup_stm32f103xb.s` |
| 链接脚本 | `STM32F103C8Tx_FLASH.ld` |
| 存储布局 | Flash 64 KiB，RAM 20 KiB |
| 调试设备名 | `STM32F103C8Tx` |
| 调试配置名 | `STM32F103C8T6 - ST-Link (SWD)` |

## 2. 快速开始

在 VS Code 中打开 `k-car.code-workspace`，或直接打开 `firmware/`。

```powershell
cd firmware
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Action Build
```

常用入口：

| 操作 | VS Code | 命令行 |
|---|---|---|
| 编译 Debug | `Ctrl+Shift+B` | `tools/build.ps1 -Action Build` |
| 编译 Release | Tasks: `STM32: Build Release` | `tools/build.ps1 -Action Build -Preset Release` |
| 重新生成 Debug 构建 | Tasks: `STM32: Reconfigure Debug` | `tools/build.ps1 -Action Configure -Preset Debug` |
| 重新生成 Release 构建 | Tasks: `STM32: Reconfigure Release` | `tools/build.ps1 -Action Configure -Preset Release` |
| 下载并复位 | Tasks: `STM32: Flash Debug (writes board)` | `tools/build.ps1 -Action Flash` |
| 枚举 ST-Link | Tasks: `STM32: List ST-Link probes` | `tools/build.ps1 -Action Probe` |
| 检查工具 | Tasks: `STM32: Check tools` | `tools/build.ps1 -Action Check` |
| 主机单元测试 | Tasks: `STM32: Host Tests (requires host gcc)` | `tests/run_tests.ps1` |

产物位于 `firmware/build/Debug/` 或 `firmware/build/Release/`：

```text
k_car.elf  # 调试和烧录
k_car.hex  # HEX
k_car.bin  # 裸二进制
k_car.map  # 链接映射
```

`Flash` 会真实写入板卡；没有确认硬件连接时只运行 `Build`。

本机当前 Debug 构建基线：

```text
FLASH: 21364 B / 64 KiB (32.60%)
RAM:    2632 B / 20 KiB (12.85%)
```

当前机器可以完成 ARM 交叉编译和 ST-Link 调试；主机单元测试需要单独的 Windows/MinGW `gcc`，当前 PATH 中未找到，所以测试任务会明确失败于“Host gcc was not found”。这不影响固件构建。

## 3. 环境配置位置

- `firmware/.vscode/launch.json`：ST-Link/SWD 调试目标，设备名已改成 `STM32F103C8Tx`，速度为 1 MHz。
- `firmware/.vscode/tasks.json`：Debug/Release 编译、重新配置、烧录、探针、工具检查和主机测试任务。
- `firmware/.vscode/settings.json`：CMake Tools、C/C++ IntelliSense、终端 PATH、构建产物排除和编译命令导出。这里含本机绝对路径。
- `firmware/.vscode/c_cpp_properties.json`：IntelliSense 使用 C8T6 宏、工程头文件、ARM/Newlib 系统头文件和 `build/Debug/compile_commands.json`；`compilerPath` 指向本机 `arm-none-eabi-gcc.exe`。
- `firmware/.vscode/extensions.json`：推荐 STM32 官方扩展、Microsoft C/C++ 和 CMake Tools。
- `firmware/tools/toolchain.local.json`：构建脚本使用的本机工具路径。
- `firmware/CMakePresets.json`：Debug/Release 预设和输出目录。
- `firmware/cmake/arm-none-eabi.cmake`：`cortex-m3`、`thumb`、软件浮点交叉编译设置。

换电脑或 STM32Cube 工具版本变化时，同时检查 `settings.json` 与
`tools/toolchain.local.json` 中的绝对路径。不要针对 C8T6 重新引入 VE/xE 的启动文件、512 KiB Flash 或 64 KiB RAM 配置。

本工程是手写 CMake，没有 STM32CubeIDE 的项目元数据。因此工作区关闭了 STM32Cube 扩展自带的 clangd，改用 Microsoft C/C++ IntelliSense；编译和 ST-Link 调试不依赖该 clangd。若 VS Code 曾显示 “Can't find a project” 或 “Could not run Clangd Language server”，重新加载窗口即可清除这类项目识别误报。

VS Code 首次打开后如果 C/C++ 仍显示大量红色波浪线，先执行一次 Debug 构建生成 `firmware/build/Debug/compile_commands.json`，再执行 `C/C++: Reset IntelliSense Database`。CMake Tools 自动配置已关闭，构建入口是 `tools/build.ps1` 和 CMake Presets，不要让扩展另外生成一套构建目录。

## 4. 当前固件

入口是 `firmware/Core/Src/main.c`。当前程序做以下事情：

1. `HAL_Init()` 初始化 HAL 与 SysTick。
2. 使用内部 HSI 8 MHz，不假设外部晶振频率。
3. `App_Init()` 初始化超声波、四路循迹/TIM3 电机、TFT/HCI 和 USART2 姿态链路。
4. 主循环调用 `App_Task()`；循迹控制按 10 ms 更新，屏幕每 100 ms 刷新。
5. 每秒递增 `heartbeat_count`；可用调试器查看 `system_clock_hz`（当前应为 `8000000`）。

`firmware/Core/Inc/stm32f1xx_hal_conf.h` 当前启用 HAL、Cortex、DMA、EXTI、Flash、GPIO、PWR、RCC、UART 模块。新增 ADC、I2C、硬件 SPI 或 HAL TIM 功能时，除了启用对应 HAL 模块，还要把驱动源文件加入 `firmware/CMakeLists.txt`。

TFT 显示五项任务数据，底部另有三行离线诊断。两圈耗时来自 MPU6050 + `LapTimer`，三个识别计数来自 `AppStatus`。USART2 使用 PA2/PA3、115200 8N1；中断只入环形缓冲，解析和浮点运算在主循环执行。四路循迹输入为 PB0/PB1/PB10/PB11，使用 PD 修正和 TIM3 PWM 电机输出；电机门控只在应用进入 `APP_DRIVE_FOLLOW` 时打开。现在正常线型连续满足 200 ms 后自动起步，普通循迹不依赖超声波；终点、故障和手动停车后锁定，详见 [firmware/AUTO_START.md](firmware/AUTO_START.md)。A/B/C/D 实际位置识别仍未实现。

工程没有 CubeMX `.ioc`，是手工组织的 CMake 工程。不要用其他板型的 CubeMX 输出直接覆盖现有目录。

## 5. 引脚基线

状态图例：`占用` = 已分配给模块；`保留` = 系统功能；`避开` = 需要关闭 JTAG 等功能后才可复用；`空闲` = 当前可用。

### 模块接线

| 模块 | 信号 | STM32 引脚 | 模式/备注 |
|---|---|---|---|
| TB6612 | PWMB，左电机调速 | PA6 | TIM3_CH1 |
| TB6612 | PWMA，右电机调速 | PA7 | TIM3_CH2 |
| TB6612 | 四个方向输入 | PB5、PB6、PB7、PB8 | PB5=AIN2、PB6=AIN1、PB7=BIN1、PB8=BIN2 |
| 四路循迹 | 输入 1～4 | PB0、PB1、PB10、PB11 | 当前按数字量使用；PB0/PB1 也有 ADC，PB10/PB11 只能数字 |
| MPU6050 串口模块 | STM32 TX → 模块 RX | PA2 | USART2_TX |
| MPU6050 串口模块 | 模块 TX → STM32 RX | PA3 | USART2_RX；115200、100 Hz，模块 SCL/SDA 不接 |
| HC-SR04 | Trig / Echo | PA4、PA5 | PA4=Trig、PA5=Echo；Echo 使用 EXTI + 定时器计时 |
| ST7735 TFT | CLK | PB14 | 软件 SPI |
| ST7735 TFT | SDI/MOSI | PB15 | 软件 SPI |
| ST7735 TFT | CS | PA1 | 普通 GPIO |
| ST7735 TFT | DC | PA12 | 普通 GPIO；使用后不再使用 USB |
| ST7735 TFT | BLK | 3.3 V | 背光常亮；可选改接 PB9 做调光 |
| ST7735 TFT | SDO/MISO | PB13 | 当前代码保留给字库/读回；只写屏可暂不接，但不要当空闲脚 |
| 蜂鸣器 | 信号 | PA0 | TIM2_CH1；无源蜂鸣器，经三极管驱动 |
| K230 | STM32 TX → K230 RX | PA9 | USART1_TX，建议 115200 |
| K230 | K230 TX → STM32 RX | PA10 | USART1_RX |
| 左编码器 | A 相 | PA8 | EXTI 计数；B 相暂不接 |
| 右编码器 | A 相 | PB12 | EXTI 计数；B 相暂不接 |

TB6612 输出侧：左电机 `M1/M2` 接 `BO1/BO2`，右电机 `M1/M2` 接 `AO1/AO2`。电机转向反了应交换该电机的两根输出线，不要用无关的 GPIO 配置去“修方向”。

所有模块必须与 STM32 共地。K230 使用独立 USB-C 5 V 供电，不要由 STM32 板给它供电。

### 引脚来源与冲突处理

用户提供的 `STM32F103C8T6_引脚占用表.md` 是硬件参考，但它和当前代码有几处需要保留的差异：

- **PB13**：引脚表写 TFT 的 SDO 可以悬空；当前代码把 PB13 预留给 TFT 字库/读回并已初始化为输入。只写显示时可以悬空，但不要把它分配给编码器 B 相、OLED 或其它模块，除非确认字库功能不使用。
- **TB6612 方向脚**：引脚表只写“PB5、PB6、PB7、PB8 中的两个”。当前代码基线是 `PB5=AIN2、PB6=AIN1、PB7=BIN1、PB8=BIN2`；上电架空车轮实测后再确认，不能仅凭表里的“中的两个”改写驱动。
- **HC-SR04**：引脚表末尾写 Trig/Echo 具体角色待确认；当前代码基线是 `PA4=Trig、PA5=Echo`。接线前用实物丝印或万用表确认。
- **TIM2 的 CH2/CH3/CH4**：引脚表把它们列为 MPU6050/TFT 的复用功能，但当前代码没有把 TIM2 用于这些功能：TFT 是软件 SPI，MPU6050 是 USART2。不要把“复用功能存在”误读成“已经被占用”。
- **PB9、PA11**：当前代码没有默认占用。PB9 只在决定给 TFT 背光调光时使用；PA11 可作普通 GPIO，但与 USB_DM/CAN_RX 复用。

遇到上述冲突时，以实际接线和 `firmware/Core/Inc/board_pins.h` 为准；改代码前先在本文记录变更原因和实测结果。

### 系统保留与空闲

| 范围 | 引脚 | 状态 |
|---|---|---|
| SWD | PA13、PA14 | 保留，分别作 SWDIO、SWCLK，勿占 |
| 启动/时钟 | PB2、PC14、PC15 | 保留，分别为 BOOT1、OSC32_IN、OSC32_OUT |
| JTAG 复用 | PA15、PB3、PB4 | 避开；复用需关闭 JTAG，当前不建议使用 |
| 板载 LED | PC13 | 仅建议做指示灯，驱动能力弱 |
| 空闲 | PA11、PB9 | PB13 已预留给 TFT 字库 |

PA11 与 USB_DM/CAN_RX 复用，PB9 与 TIM4_CH4/可选 TFT 背光复用。若 PB9 用于背光，则只剩 PA11 一个普通空闲脚（不含 JTAG 复用脚）。PB13 预留给 TFT 字库。

## 6. 未确认事项

以下内容在原引脚表里没有定死。后续 agent 应先查原理图、实物丝印或代码注释，不要把推测写进驱动：

1. **TB6612 配对是当前代码基线，不是原始引脚表的明确结论。** PB5=AIN2、PB6=AIN1、PB7=BIN1、PB8=BIN2；仍需实测左右电机和前进电平。
2. **HC-SR04 角色是当前代码基线。** PA4=Trig、PA5=Echo；原始引脚表写“待确认”，接线前必须用实物确认。
3. **编码器只有 A 相已接线。** PA8、PB12 只能做脉冲计数，当前无法从编码器判断方向，也不能做硬件正交解码。若需要方向，必须重新规划 B 相和/或定时器资源。
4. **外部引脚表末尾的 OLED 备注已过时。** PB12 已分配给右编码器 A 相，PB13 预留给 TFT 字库，不应再占用。
5. **TFT 背光默认常亮。** 只有决定使用 PB9 调光时，才把它从“空闲”改为 TFT 背光。
6. **时钟尚未使用外部晶振。** 若后续要跑 72 MHz，应先确认板上 HSE 晶振频率和起振情况，再改 PLL/Flash latency。
7. **PB13 的“可悬空”和“字库保留”存在文档冲突。** 只写屏可以悬空，但当前代码仍需要它作为字库/读回输入；确认字体芯片方案前不要改作他用。

## 7. 资源约束

- **没有可直接使用的硬件 I2C。** I2C1 的 PB6/PB7 被 TB6612 占用；I2C2 的 PB10/PB11 被循迹占用。OLED 等设备应使用软件 I2C，并先选空闲脚。
- **硬件 SPI 基本不可用。** SPI1 的关键脚被电机、超声波占用；SPI2 的 PB14/PB15 被 TFT 占用，PB12 也已占用。TFT 继续使用软件 SPI。
- **没有额外的硬件串口。** USART1 给 K230，USART2 给 MPU6050，USART3 的 PB10/PB11 给循迹，不能再假设有第四路硬件 UART。
- **PB10/PB11 没有 ADC。** 若循迹模块是模拟灰度输出，只有 PB0/PB1 可做 ADC，另外两路需要改接或使用外部 ADC/比较器。
- **定时器通道引脚很紧。** 当前实际使用是 TIM3_CH1/CH2 输出两路电机 PWM、TIM2_CH1 驱动蜂鸣器、TIM4 做超声波计时；PA8/PB12 只是 EXTI 编码器 A 相，不是硬件编码器模式。PA4/PA5 也没有输入捕获通道。需要 PWM/捕获/编码器时先核对引脚与定时器映射，不要只看定时器编号，也不要把引脚表的复用功能列当成实际占用。
- **C8T6 只有 20 KiB RAM。** 视觉协议缓冲、TFT 显存和滤波数组都要控制大小，禁止直接套用大容量板的内存布局。

## 8. 硬件安全注意

- HC-SR04 的 Echo 通常是 5 V 输出。先确认所用 C8T6 板卡和目标脚的电平耐受，优先使用电阻分压或电平转换，不要直接试验性接 5 V。
- 编码器、MPU6050、TFT 等模块优先用 3.3 V 逻辑；若模块由 5 V 供电，必须确认其信号输出不会超过 STM32 引脚容许电压。
- TB6612 的 VM 电机电源与 VCC 逻辑电源分开，务必共地；首次测试应架空轮子。
- 烧录和调试必须保留 PA13/PA14。BOOT0 正常启动时为 0。

## 9. 推荐工作顺序

1. 先确认 `Ctrl+Shift+B` 可编译，再用 ST-Link 验证连接和下载。
2. 在架空车轮的条件下验证 TB6612 的左右电机、方向脚配对和 PWM 极性。
3. 超声波、TFT/HCI、MPU6050、蜂鸣器、四路循迹和电机控制已接入；已加入稳定黑线自动发车；下一步实现位置事件、参数标定、实体停止按键、K230 和停车动作。
4. 每完成一个模块，同步更新本文的接线表、未确认事项和 HAL/CMake 模块列表。
5. 修改引脚前先检查 `firmware/Core`、`firmware/CMakeLists.txt`、`firmware/.vscode/launch.json` 和本文，避免代码、构建与文档三套配置漂移。

## 10. 目录说明

```text
.
├── README.md                   # 本文件：项目与 agent 上手基线
├── k-car.code-workspace        # VS Code 工作区
├── firmware/
│   ├── CMakeLists.txt          # 目标、源文件和内存布局入口
│   ├── CMakePresets.json       # Debug/Release 预设
│   ├── STM32F103C8Tx_FLASH.ld  # C8T6 链接脚本
│   ├── Core/                   # 启动、状态机、HMI、姿态、超声波和 HAL 配置
│   ├── Drivers/                # ST HAL/CMSIS 第三方源码
│   ├── tools/                  # 本机构建、烧录、探针脚本
│   └── .vscode/                # 构建、调试、IntelliSense 配置
├── output/                     # 采购清单等输出物
├── _k_figs/、tmp/、work/       # 文档图片、PDF 和中间资料
└── build_k_bom_pdf.py          # 采购清单 PDF 生成脚本
```

固件构建细节见 [firmware/README.md](firmware/README.md)。
