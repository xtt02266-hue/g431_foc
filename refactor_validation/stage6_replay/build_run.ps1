# Stage 6 A/B replay: compile the Stage 0 reference scheduler and the current
# migrated Motor_SpeedEstimator_UpdateAdaptive, run identical timelines, compare.
param(
    [string]$Py = 'C:\Users\scott\AppData\Local\Temp\opencode\stage5_verif\.venv\Scripts\python.exe'
)
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$root = (Resolve-Path (Join-Path $here '..\..')).Path
$shim = Join-Path $here 'shim'
$common = @((Join-Path $here 'support.c'), (Join-Path $here 'driver.c'))
$speed = Join-Path $root 'user\src\motor_speed_loop.c'
$incs = @("-I$shim", "-I$($root)\user\inc", "-I$($root)\Core\Inc")

function Build([string[]]$extra, [string]$out) {
    $a = @('-m', 'ziglang', 'cc', '-std=c11', '-O0', '-w') + $incs + $common + $extra + @($speed, '-o', $out)
    & $Py @a
    if ($LASTEXITCODE -ne 0) { throw "build failed: $out" }
}

$refExe = Join-Path $here 'out_ref.exe'
$newExe = Join-Path $here 'out_new.exe'
Build @((Join-Path $here 'ref_impl.c')) $refExe
Build @((Join-Path $here 'new_impl.c')) $newExe

$refTxt = Join-Path $here 'result_ref.txt'
$newTxt = Join-Path $here 'result_new.txt'
& $refExe | Set-Content -Encoding ASCII $refTxt
& $newExe | Set-Content -Encoding ASCII $newTxt

$d = Compare-Object (Get-Content $refTxt) (Get-Content $newTxt)
if ($null -eq $d) {
    Write-Output "STAGE6 A/B: IDENTICAL ($((Get-Content $refTxt).Count) lines)"
} else {
    Write-Output "STAGE6 A/B: DIFFERENT"
    $d | Format-Table -AutoSize | Out-String | Write-Output
    exit 2
}
