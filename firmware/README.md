# K 题 STM32F103C8T6 固件工程

硬件目标：STM32F103C8T6，Cortex-M3，64 KiB Flash、20 KiB RAM，ST-Link/SWD。
使用 STM32CubeF1 V1.8.7 HAL/CMSIS、GNU Arm GCC、CMake、Ninja 和 ST 官方 VS Code 扩展。

项目级的引脚基线、资源限制和未确认事项见上一级目录的 `README.md`。

## 开始开发

在 VS Code 打开本目录，或打开上一级的 `k-car.code-workspace`。

- `Ctrl+Shift+B`：编译 Debug，生成 `build/Debug/k_car.elf`、`.hex`、`.bin` 和 `.map`。
- `F5`：先编译，再通过 ST-Link 下载并停在 `main`。继续运行后暂停，可查看变量。
- 命令面板 → `Tasks: Run Task` → `STM32: Flash Debug (writes board)`：编译、下载、校验并复位。
- `STM32: List ST-Link probes`：仅枚举下载器。
- `STM32: Check tools`：检查本机工具版本。
- Release 编译通过任务 `STM32: Build Release` 完成。

命令行也可运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Action Build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Action Build -Preset Release
```

工具路径记录在 `tools/toolchain.local.json`，VS Code 的补全、编译及调试路径位于
`.vscode/settings.json` 和 `.vscode/launch.json`。换电脑时需要重新配置这些本机路径。

## 当前程序

`Core/Src/main.c` 使用内部 HSI 8 MHz 时钟，不依赖开发板外部晶振。`App_Init()`
会初始化 HC-SR04、ST7735/HCI 和 USART2 姿态模块；`App_Task()` 在主循环中测距、
接收 MPU6050 角度帧、运行圈计时并每 100 ms 刷新屏幕。蜂鸣器已接入，说明见 [BUZZER.md](BUZZER.md)。电机、循迹和视觉仍未实现。

已加入比赛任务状态机与模块占位接口，详见 [FRAMEWORK.md](FRAMEWORK.md)。
HC-SR04 说明见 [ULTRASONIC.md](ULTRASONIC.md)，人机界面与串口姿态链路见
[HMI_ATTITUDE.md](HMI_ATTITUDE.md)。当前默认等待发车，因为发车按键和有效循迹输入
仍是占位接口；不要据此认为小车已经能自动跑圈。保留 PA13/PA14 的 SWD 调试功能。

工程使用 `STM32F103xB` 器件宏、`startup_stm32f103xb.s` 和
`STM32F103C8Tx_FLASH.ld`。本工程手工组织 CMake 和 HAL 源文件，当前不包含 CubeMX `.ioc`；不要把其他板型的
CubeMX 输出直接覆盖到这里。后续如使用 CubeMX，可按确切硬件配置生成 CMake 工程再合并。

## ST-Link 连接

SWDIO 对应 PA13，SWCLK 对应 PA14，GND 共地。开发板需要正常的 3.3 V 供电。
具有 VTref 引脚的 ST-Link 应将 VTref 接目标板 3.3 V；不同 ST-Link 的电源脚定义
可能不同，以下载器丝印/说明为准。NRST 可连接以支持硬件复位和复位下连接。
默认调试采用软件复位、SWD 1 MHz，BOOT0 正常启动时设为 0。

目前软件侧编译与硬件侧运行是两项独立验证；电脑未连接板卡时无法验证下载和断点。

## 源码来源

第三方文件复制自本机官方固件包：
`STM32Cube/Repository/STM32Cube_FW_F1_V1.8.7`。

- `Drivers/STM32F1xx_HAL_Driver`：ST HAL/LL 驱动，保留原许可证。
- `Drivers/CMSIS`：Arm CMSIS 和 ST F1 设备头文件，保留各自许可证。
- `Core/Startup/startup_stm32f103xb.s`、`Core/Src/system_stm32f1xx.c`：
  ST 官方设备模板，许可证见 `Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt`。
- HAL 配置由官方模板裁剪启用模块；链接脚本按 F103C8T6 调整为 64 KiB Flash、20 KiB RAM。

官方参考：
- https://www.st.com/en/microcontrollers-microprocessors/stm32f103c8.html
- https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/latest/en/docs/markup/getting_started/installation.html
