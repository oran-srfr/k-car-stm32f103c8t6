# Four-sensor line following

## Wiring used by this project

| Signal | STM32 pin | Notes |
|---|---|---|
| X1 input | PB0 | Digital input, pull-up |
| X2 input | PB1 | Digital input, pull-up |
| X3 input | PB10 | Digital input, pull-up |
| X4 input | PB11 | Digital input, pull-up |
| Left PWM, PWMB | PA6 | TIM3_CH1 |
| Right PWM, PWMA | PA7 | TIM3_CH2 |
| Left BIN1 / BIN2 | PB7 / PB8 | TB6612 direction |
| Right AIN1 / AIN2 | PB6 / PB5 | TB6612 direction |
| TB6612 STBY | 3.3 V | Wired high; not an MCU pin |

The attached module originally used PA0..PA3 for sensors, PB0/PB1/PB10/PB11
for motor direction, and PB12 for STBY. Those assignments conflict with the
buzzer, TFT, attitude UART, tracking inputs, and right encoder in this project,
so only its control algorithms were imported. The hardware mapping was replaced
with the confirmed wiring above.

`line_track.h` assumes a black line produces a low sensor output. Change
`LINE_SENSOR_BLACK_ACTIVE_LOW` to `0U` if the installed modules output high over
black. The physical left-to-right sensor order must also be checked on the car;
the current geometry is X2, X1, X3, X4.

## Control stack

```text
PB0/PB1/PB10/PB11
  -> line_sensor.c     raw GPIO levels
  -> line_track.c      black bitmap and weighted deviation
  -> pd_controller.c   proportional/derivative correction
  -> motor_command.c   left/right software commands
  -> line_follow.c     tracking, search, reacquire and fault state machine
  -> vehicle_control.c output safety gate
  -> motor_driver.c    TIM3 PWM and TB6612 direction pins
```

The source files were adapted from the supplied `xunji_c8t6` project. The
control period is 10 ms. `Modules_Poll()` calls `VehicleControl_Update()` at
that period and publishes its result as `AppInputs.line_valid`.

`Modules_Drive(APP_DRIVE_FOLLOW)` enables the motor gate. STOP, obstacle hold,
parking, fault, and finish states disable it. The line-follow algorithm keeps
sampling while the gate is closed, but PWM and direction outputs remain zero.
If the board boots before it is placed on the track, the follower may latch a
`LINE_FOLLOW_FAULT_NO_VALID_HISTORY`. Detection of a valid normal line
automatically re-arms the follower while the motor gate stays closed.

## Tunable defaults

`control_config.h` contains provisional values copied from the supplied project:

- `Kp = 1.0`, `Kd = 0.0`, correction limit `10.0`
- base speed `20.0`, maximum forward command `100.0`
- curve speed `16.0`, sharp-curve speed `12.0`, search speed `8.0`
- 30 lost cycles before a fault, 10 cycles for reacquisition

These are software command units between 0 and 1000, not physical speed or a
calibrated PWM percentage. The defaults are not validated on the assembled car
and are likely too low for final operation. Tune them only with the wheels raised
first, then on a test track.

`MOTOR_DRIVER_LEFT_REVERSED` and `MOTOR_DRIVER_RIGHT_REVERSED` in
`motor_driver.h` must be used to correct motor polarity. Do not swap application
left/right semantics to compensate for wiring.

## Debug variables

- `g_tracking_status`, `g_tracking_bits`, `g_tracking_deviation`
- `g_vehicle_control_state`
- `g_left_target_command`, `g_right_target_command`
- `g_left_applied_command`, `g_right_applied_command`
- `g_motor_driver_left_pwm_compare`, `g_motor_driver_right_pwm_compare`

## Current limits

- Startup requires a normal black-line pattern continuously for 200 ms; sonar is
  not part of the start condition. See [AUTO_START.md](AUTO_START.md).
  Finished/fault/stopped tasks need explicit re-arming. A/B/C/D position events
  still have no real source.
- A/B/C/D lap completion is not inferred from line sensors or yaw.
- Vision, parking trajectories, and encoders are still unimplemented.
- The TB6612 STBY pin is hardwired high, so firmware cannot provide a physical
  standby shutdown. The driver still forces PWM and direction outputs to zero
  on every stop/fault path.
- Hardware behavior has not been verified on the car.

## Verification

The Debug and Release firmware builds compile the full line-follow and motor
stack. `tests/test_tracking.c` covers sensor geometry, PD behavior, and motor
command mixing. Run `tests/run_tests.ps1` when a host `gcc` is available.
