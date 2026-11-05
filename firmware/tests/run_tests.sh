#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")/.."
out="${TEST_OUTPUT_DIR:-build/host}"
mkdir -p "$out"
run_test() {
 name="$1"; shift
 "${CC:-gcc}" -std=c11 -Wall -Wextra -Werror -O1 -ICore/Inc "$@" -o "$out/$name" -lm
 "$out/$name"
}
run_test test_app Core/Src/app.c tests/test_app.c
run_test test_ultrasonic Core/Src/ultrasonic.c tests/test_ultrasonic.c
run_test test_buzzer Core/Src/buzzer.c tests/test_buzzer.c
run_test test_laptimer Core/Src/LapTimer.c tests/test_laptimer.c
run_test test_mpulink -Itests Core/Src/MpuLink.c tests/test_mpulink.c
run_test test_tracking Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c tests/test_tracking.c
run_test test_line_follow_pd -Itests -DLINE_FOLLOW_DISCRETE_MODE=0 -DLINE_FOLLOW_TARGET_DEVIATION=0.0f -DLINE_FOLLOW_CORNER_ENABLE=0 Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c tests/test_line_follow.c
run_test test_discrete_line_follow -Itests -DLINE_FOLLOW_DISCRETE_MODE=1 -DLINE_FOLLOW_CORNER_ENABLE=0 Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c tests/test_discrete_line_follow.c
run_test test_point_beep -Itests Core/Src/point_beep.c tests/test_point_beep.c
run_test test_corner -Itests -DLINE_FOLLOW_DISCRETE_MODE=0 -DLINE_FOLLOW_TARGET_DEVIATION=0.0f Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c tests/test_corner.c
run_test test_edge_target -Itests -DLINE_FOLLOW_DISCRETE_MODE=0 Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c tests/test_edge_target.c
run_test test_control_timing -Itests Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c Core/Src/vehicle_control.c tests/test_control_timing.c
run_test test_obstacle_resume -Itests Core/Src/app.c Core/Src/line_track.c Core/Src/pd_controller.c Core/Src/motor_command.c Core/Src/line_control.c Core/Src/line_follow.c Core/Src/vehicle_control.c tests/test_obstacle_resume.c
