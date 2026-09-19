param(
    [Parameter(Mandatory = $true)][string]$Baseline,
    [string]$CompilerPython = 'python'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$scratch = Join-Path $env:TEMP 'sensorless-ab-20260919'
New-Item -ItemType Directory -Path $scratch -Force | Out-Null

function BuildAndRun([string]$source, [string]$name) {
    $exe = Join-Path $scratch "$name.exe"
    $output = Join-Path $scratch "$name.txt"
    & $CompilerPython -m ziglang cc -std=c11 -O0 `
        "-I$PSScriptRoot" "-I$(Join-Path $root 'user\inc')" `
        (Join-Path $PSScriptRoot 'replay.c') $source -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $name" }
    & $exe | Set-Content -Encoding ASCII $output
    if ($LASTEXITCODE -ne 0) { throw "replay coverage failed: $name" }
    return $output
}

$before = BuildAndRun (Resolve-Path $Baseline).Path 'before'
$after = BuildAndRun (Join-Path $root 'user\src\motor_sensorless.c') 'after'
$delta = Compare-Object (Get-Content $before) (Get-Content $after)
if ($null -ne $delta) {
    $delta | Select-Object -First 12 | Format-Table | Out-String | Write-Output
    throw 'sensorless A/B replay differs'
}
Write-Output "sensorless A/B replay identical: $((Get-Content $after).Count) samples"
