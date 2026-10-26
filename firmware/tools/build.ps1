param(
    [ValidateSet('Build', 'Configure', 'Flash', 'Probe', 'Check')]
    [string]$Action = 'Build',
    [ValidateSet('Debug', 'Release')]
    [string]$Preset = 'Debug'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$configPath = Join-Path $PSScriptRoot 'toolchain.local.json'
if (-not (Test-Path -LiteralPath $configPath)) {
    throw 'Missing tools/toolchain.local.json. Configure local tool paths first.'
}
$tools = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
foreach ($name in @('cmake', 'ninja', 'gcc', 'gdb', 'programmer')) {
    if (-not (Test-Path -LiteralPath $tools.$name)) {
        throw "Tool missing: $name = $($tools.$name)"
    }
}
$toolDirs = @($tools.cmake, $tools.ninja, $tools.gcc, $tools.programmer) |
    ForEach-Object { Split-Path $_ -Parent } | Select-Object -Unique
$previousPath = $env:PATH
$env:PATH = ($toolDirs -join ';') + ';' + $env:PATH

function Invoke-Checked {
    param([string]$Executable, [string[]]$ToolArguments)
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed (exit $LASTEXITCODE): $Executable $ToolArguments"
    }
}

Push-Location $projectRoot
try {
    if ($Action -eq 'Check') {
        Invoke-Checked $tools.gcc @('--version')
        Invoke-Checked $tools.gdb @('--version')
        Invoke-Checked $tools.cmake @('--version')
        Invoke-Checked $tools.ninja @('--version')
        Write-Host 'STM32 tool paths are valid. Board connection is a separate check.'
        return
    }
    if ($Action -eq 'Probe') {
        Invoke-Checked $tools.programmer @('-l', 'stlink')
        return
    }
    Invoke-Checked $tools.cmake @('--preset', $Preset)
    if ($Action -eq 'Configure') { return }
    Invoke-Checked $tools.cmake @('--build', '--preset', $Preset, '--parallel')
    if ($Action -eq 'Flash') {
        # This action intentionally writes firmware; it is never run by Build.
        $elf = Join-Path $projectRoot "build/$Preset/k_car.elf"
        Invoke-Checked $tools.programmer @('-c', 'port=SWD', 'freq=1000', '-w', $elf, '-v', '-rst')
    }
} finally {
    Pop-Location
    $env:PATH = $previousPath
}
