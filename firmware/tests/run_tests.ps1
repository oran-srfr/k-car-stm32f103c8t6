$ErrorActionPreference = 'Stop'

$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $gcc) {
    Write-Host 'Host gcc was not found. Install MinGW or add gcc to PATH.' -ForegroundColor Red
    exit 1
}

Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    $out = Join-Path $PWD 'build/host'
    New-Item -ItemType Directory -Path $out -Force | Out-Null
    $failed = 0

    function Invoke-Test {
        param(
            [string]$Name,
            [string[]]$Sources,
            [string[]]$Extra = @()
        )

        Write-Host ""
        Write-Host "==> $Name"
        $exe = Join-Path $out "$Name.exe"
        $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-O1',
                       '-ICore/Inc') + $Extra + $Sources + @('-o', $exe, '-lm')
        & $gcc.Source @arguments
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Compile failed: $Name" -ForegroundColor Red
            $script:failed++
            return
        }
        & $exe
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Test failed: $Name" -ForegroundColor Red
            $script:failed++
        }
    }

    Invoke-Test -Name 'test_app' -Sources @('Core/Src/app.c', 'tests/test_app.c')
    Invoke-Test -Name 'test_ultrasonic' -Sources @('Core/Src/ultrasonic.c', 'tests/test_ultrasonic.c')
    Invoke-Test -Name 'test_buzzer' -Sources @('Core/Src/buzzer.c', 'tests/test_buzzer.c')
    Invoke-Test -Name 'test_laptimer' -Sources @('Core/Src/LapTimer.c', 'tests/test_laptimer.c')
    Invoke-Test -Name 'test_mpulink' -Sources @('Core/Src/MpuLink.c', 'tests/test_mpulink.c') -Extra @('-Itests')
    Invoke-Test -Name 'test_tracking' -Sources @('Core/Src/line_track.c', 'Core/Src/pd_controller.c', 'Core/Src/motor_command.c', 'tests/test_tracking.c')
    Invoke-Test -Name 'test_line_follow_pd' -Sources @('Core/Src/line_track.c', 'Core/Src/pd_controller.c', 'Core/Src/motor_command.c', 'Core/Src/line_control.c', 'Core/Src/line_follow.c', 'tests/test_line_follow.c') -Extra @('-Itests', '-DLINE_FOLLOW_DISCRETE_MODE=0')
    Invoke-Test -Name 'test_discrete_line_follow' -Sources @('Core/Src/line_track.c', 'Core/Src/pd_controller.c', 'Core/Src/motor_command.c', 'Core/Src/line_control.c', 'Core/Src/legacy_line_walking.c', 'Core/Src/line_follow.c', 'tests/test_discrete_line_follow.c')

    if ($script:failed -ne 0) {
        Write-Host "$($script:failed) test executable(s) failed." -ForegroundColor Red
        exit 1
    }
    Write-Host 'All host tests passed.' -ForegroundColor Green
} finally {
    Pop-Location
}
