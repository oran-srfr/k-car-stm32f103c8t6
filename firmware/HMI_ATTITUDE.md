# HMI and serial attitude integration

## Hardware

| Module | Signal | STM32 pin | Notes |
|---|---|---|---|
| ST7735 TFT | SCL/CLK | PB14 | Software SPI |
| ST7735 TFT | SDI/MOSI | PB15 | Software SPI |
| ST7735 TFT | CS | PA1 | Shared with the font ROM |
| ST7735 TFT | DC | PA12 | Command/data |
| ST7735 TFT | SDO/MISO | PB13 | Font ROM readback |
| ST7735 TFT | BLK | 3.3 V | Always on |
| MPU6050 serial module | module TX -> MCU RX | PA3 | USART2_RX |
| MPU6050 serial module | MCU TX -> module RX | PA2 | USART2_TX |

The display uses a 128x160 portrait layout. It shows two lap times and the
obstacle, reverse-parking, and side-parking counters.

## Data flow

```text
LCD_Init() -> HCI_Init()

USART2 RX interrupt
  -> MpuLink ring buffer
  -> Attitude_Poll()
  -> MpuLink frame parser
  -> Yaw queue
  -> Lap_FeedYaw()

App_Task()
  -> Modules_Display()
  -> copy gyro lap times and application counters into HCI_*
  -> HCI_Update()
```

The serial parser accepts the external attitude module's 11-byte angle frame:

```text
55 53 RollL RollH PitchL PitchH YawL YawH VersionL VersionH SUM
```

`SUM` is the low byte of the sum of the first ten bytes. Yaw is little-endian
and is converted to the range -180 to +180 degrees.

## Source files

- `Core/Src/Attitude.c`, `Core/Inc/Attitude.h`: USART2 hardware setup and the
  integrated yaw/lap status API.
- `Core/Src/MpuLink.c`, `Core/Inc/MpuLink.h`: interrupt-safe byte ring,
  streaming frame parser, checksum, and Yaw queue.
- `Core/Src/LapTimer.c`, `Core/Inc/LapTimer.h`: unwrap, slow-rotation
  accumulation, dropout rejection, and two-lap timing.
- `Core/Src/HCI.c`, `Core/Inc/HCI.h`: display refresh and five displayed fields.
- `Core/Src/lcd*.c`, `Core/Src/zk.c`: ST7735 and GB2312 font-ROM drivers.
- `Core/Src/app_modules.c`: integration with the existing application framework.

## Runtime behavior

- USART2 is fixed at 115200 baud, 8N1. The example project's baud-rate probing
  was intentionally not copied, because the confirmed module configuration is
  115200 and probing delays startup.
- The UART interrupt only stores one received byte into a 128-byte ring buffer
  and re-arms reception. Parsing and floating-point work run in the main loop.
- Starting a lap-mode run resets and starts `LapTimer`. Stop, fault, and finish
  states stop the timer. Parking modes do not start a lap.
- HMI lap fields come from `LapTimer`. The counter fields come from `AppStatus`.
- The application state machine still determines lap completion from geometric
  A/B/C/D events. `LapTimer` does not replace that state machine, and a 720-degree
  rotation alone is not treated as proof that the car reached point A.
- The provided example's `StartLine` and `RunStart` modules were not copied.
  They would duplicate the current application's start and lap arbitration.
- No start button, tracking algorithm, motor output, or vision parser
  implementation is added by this integration.

## Debugging

`Attitude_GetStatus()` exposes:

- `ready`: USART2 initialized successfully.
- `live`: a valid frame arrived within 500 ms.
- `yaw_deg`, `yaw_ms`: most recently delivered angle.
- `link`: byte/frame/overflow/drop statistics.
- `lap`: running, finished, lap count, lap times, accumulated angle, and rate.

If bytes arrive but `frames_ok` stays zero, check baud rate and whether the
module really emits `0x55 0x53` angle frames. If `ring_overflow` grows, the main
loop is spending too long without polling.

## Verification

Debug and Release firmware builds include all new sources. The host tests are in
`tests/test_laptimer.c` and `tests/test_mpulink.c`; run them with
`tests/run_tests.ps1` when a host `gcc` is available.

The current machine has no host C compiler, so the test sources were compiled
with the ARM compiler but not executed. Display, UART framing, and deck behavior
still require board-level verification.
