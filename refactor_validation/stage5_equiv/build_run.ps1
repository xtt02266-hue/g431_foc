# Stage 5 A/B behavior-equivalence check.
# Compile baseline and current motor_system.c into host executables with zig cc,
# run both with the same deterministic driver, and compare line by line.
param(
    [string]$Py = 'C:\Users\scott\AppData\Local\Temp\opencode\stage5_verif\.venv\Scripts\python.exe'
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$root = (Resolve-Path (Join-Path $here '..\..')).Path
$shim = Join-Path $here 'shim'

function Build-Variant([string]$src, [string]$out) {
    $buildArgs = @(
        '-m', 'ziglang', 'cc', '-std=c11', '-O0', '-w',
        "-I$shim", "-I$($root)\user\inc", "-I$($root)\Core\Inc",
        (Join-Path $here 'test_support.c'),
        (Join-Path $here 'driver.c'),
        $src,
        '-o', $out
    )
    & $Py @buildArgs
    if ($LASTEXITCODE -ne 0) { throw "build failed: $src" }
}

$baselineSrc = Join-Path $root 'refactor_validation\baseline\user\src\motor_system.c'
$currentSrc  = Join-Path $root 'user\src\motor_system.c'
$baselineExe = Join-Path $here 'out_baseline.exe'
$currentExe  = Join-Path $here 'out_current.exe'

Build-Variant $baselineSrc $baselineExe
Build-Variant $currentSrc  $currentExe

$baselineTxt = Join-Path $here 'result_baseline.txt'
$currentTxt  = Join-Path $here 'result_current.txt'

& $baselineExe | Set-Content -Encoding ASCII $baselineTxt
& $currentExe  | Set-Content -Encoding ASCII $currentTxt

$b = Get-Content $baselineTxt
$c = Get-Content $currentTxt
$diff = Compare-Object $b $c
if ($null -eq $diff) {
    Write-Output "A/B RESULT: IDENTICAL ($($b.Count) lines)"
} else {
    Write-Output "A/B RESULT: DIFFERENT"
    $diff | Format-Table -AutoSize | Out-String | Write-Output
    exit 2
}
