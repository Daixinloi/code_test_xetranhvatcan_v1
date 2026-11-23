# Sweep the fixed-line detour offset and hold distance against the two sim cases.
# nominal must keep "finish" at 14.37 s (no trigger fires on the empty course);
# t9 must stop touching the extra obstacle.
# Note: the earlier runs/sweep_offset.ps1 regex used "f$" which never matches a
# CRLF file, so it silently swept one value. This version matches "f" only.
$ErrorActionPreference = 'Stop'
$root = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1'
$nav  = Join-Path $root 'code_test_xetranhvatcan\Src\navigation.c'
$proj = Join-Path $root 'code_test_xetranhvatcan'
$out  = Join-Path $root 'runs\20261123-waypoint-avoid\sweep_offset_hold.txt'

Copy-Item -LiteralPath $nav -Destination ($nav + '.bak') -Force
$orig = Get-Content -LiteralPath $nav -Raw
Set-Content -LiteralPath $out -Value @('# off hold | NOM | T9   (t9 PASS = no contact, outcome finish)') -Encoding utf8
try {
    foreach ($off in @(35, 50, 65)) {
        foreach ($hold in @(260, 400, 540)) {
            $p = $orig -replace '(?m)^#define FIXED_LINE_OFFSET_MM [0-9.]+f', ("#define FIXED_LINE_OFFSET_MM " + $off + ".0f")
            $p = $p     -replace '(?m)^#define FIXED_LINE_HOLD_MM [0-9.]+f',   ("#define FIXED_LINE_HOLD_MM " + $hold + ".0f")
            Set-Content -LiteralPath $nav -Value $p -NoNewline
            Push-Location $proj
            gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_sweep.exe 2>$null | Out-Null
            if ($LASTEXITCODE -ne 0) {
                Pop-Location
                Add-Content -LiteralPath $out -Value "off=$off hold=$hold COMPILE_FAIL"
                continue
            }
            $nom = (& '.\build\sim_sweep.exe' trace nominal 2>$null | Select-Object -Last 1)
            $t9  = (& '.\build\sim_sweep.exe' trace t9      2>$null | Select-Object -Last 1)
            Pop-Location
            Add-Content -LiteralPath $out -Value "off=$off hold=$hold | NOM=$nom | T9=$t9"
        }
    }
} finally {
    Copy-Item -LiteralPath ($nav + '.bak') -Destination $nav -Force
    Remove-Item -LiteralPath ($nav + '.bak') -Force
    Add-Content -LiteralPath $out -Value 'DONE'
}
