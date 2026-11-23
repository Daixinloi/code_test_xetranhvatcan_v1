# Run the whole sim_course suite twice: once with the detour trigger active and
# once with FIXED_LINE_SIDE_MM forced to 0 (trigger disabled). Comparing the two
# per-case outcome lines shows whether the new trigger changed any other case.
# The suite aborts at its first failing CHECK. That CHECK is pre-existing: the
# reference-route checks assume the A* route, but the course now runs the
# fixed-line table. So this script records each run's first failure line rather
# than throwing on the non-zero exit code.
$ErrorActionPreference = 'Continue'
$root = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1'
$nav  = Join-Path $root 'code_test_xetranhvatcan\Src\navigation.c'
$proj = Join-Path $root 'code_test_xetranhvatcan'
$dir  = Join-Path $root 'runs\20261123-waypoint-avoid'

Copy-Item -LiteralPath $nav -Destination ($nav + '.bak') -Force
$orig = Get-Content -LiteralPath $nav -Raw
try {
    Push-Location $proj
    # Active trigger (current source).
    gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_on.exe 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'compile failed (trigger on)' }
    & '.\build\sim_on.exe' 1> (Join-Path $dir 'suite_trigger_on_out.txt') 2> (Join-Path $dir 'suite_trigger_on_err.txt')
    "ON_EXIT=$LASTEXITCODE" | Add-Content -LiteralPath (Join-Path $dir 'suite_trigger_on_err.txt')

    # Trigger disabled.
    Set-Content -LiteralPath $nav -Value ($orig -replace '(?m)^#define FIXED_LINE_SIDE_MM 100\.0f', '#define FIXED_LINE_SIDE_MM 0.0f') -NoNewline
    gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_off.exe 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'compile failed (trigger off)' }
    & '.\build\sim_off.exe' 1> (Join-Path $dir 'suite_trigger_off_out.txt') 2> (Join-Path $dir 'suite_trigger_off_err.txt')
    "OFF_EXIT=$LASTEXITCODE" | Add-Content -LiteralPath (Join-Path $dir 'suite_trigger_off_err.txt')
    Pop-Location
} finally {
    Copy-Item -LiteralPath ($nav + '.bak') -Destination $nav -Force
    Remove-Item -LiteralPath ($nav + '.bak') -Force
}
