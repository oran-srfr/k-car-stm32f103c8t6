#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/.."
out="${TEST_OUTPUT_DIR:-build/host}"
mkdir -p "$out"
run_test() {
    name="$1"
    shift
    "${CC:-gcc}" -std=c11 -Wall -Wextra -Werror -O1 -ICore/Inc -Itests "$@" -o "$out/$name" -lm
    "$out/$name"
}
run_test test_app Core/Src/app.c tests/test_app.c
run_test test_ultrasonic Core/Src/ultrasonic.c tests/test_ultrasonic.c
run_test test_buzzer Core/Src/buzzer.c tests/test_buzzer.c
run_test test_laptimer Core/Src/LapTimer.c tests/test_laptimer.c
run_test test_mpulink Core/Src/MpuLink.c tests/test_mpulink.c
run_test test_tracking Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c tests/test_tracking.c
run_test test_line_follow_pd -DLINE_FOLLOW_DISCRETE_MODE=0 Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c tests/test_line_follow.c
run_test test_discrete_line_follow Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/legacy_line_walking.c Core/Src/line_follow.c tests/test_discrete_line_follow.c
