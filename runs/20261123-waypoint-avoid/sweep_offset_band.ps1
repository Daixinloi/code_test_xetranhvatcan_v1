# Map the usable offset band at the chosen hold distance, so the tuning is not a
# knife edge. Same two cases as sweep_offset_hold.ps1.
$ErrorActionPreference = 'Stop'
$root = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1'
$nav  = Join-Path $root 'code_test_xetranhvatcan\Src\navigation.c'
$proj = Join-Path $root 'code_test_xetranhvatcan'
$out  = Join-Path $root 'runs\20261123-waypoint-avoid\sweep_offset_band.txt'

Copy-Item -LiteralPath $nav -Destination ($nav + '.bak') -Force
$orig = Get-Content -LiteralPath $nav -Raw
Set-Content -LiteralPath $out -Value @('# hold=400; offset band') -Encoding utf8
try {
    foreach ($off in @(20, 25, 30, 33, 35, 37, 40, 45)) {
        $p = $orig -replace '(?m)^#define FIXED_LINE_OFFSET_MM [0-9.]+f', ("#define FIXED_LINE_OFFSET_MM " + $off + ".0f")
        $p = $p     -replace '(?m)^#define FIXED_LINE_HOLD_MM [0-9.]+f',   "#define FIXED_LINE_HOLD_MM 400.0f"
        Set-Content -LiteralPath $nav -Value $p -NoNewline
        Push-Location $proj
        gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_sweep.exe 2>$null | Out-Null
        if ($LASTEXITCODE -ne 0) {
            Pop-Location
            Add-Content -LiteralPath $out -Value "off=$off COMPILE_FAIL"
            continue
        }
        $nom = (& '.\build\sim_sweep.exe' trace nominal 2>$null | Select-Object -Last 1)
        $t9  = (& '.\build\sim_sweep.exe' trace t9      2>$null | Select-Object -Last 1)
        Pop-Location
        Add-Content -LiteralPath $out -Value "off=$off | NOM=$nom | T9=$t9"
    }
} finally {
    Copy-Item -LiteralPath ($nav + '.bak') -Destination $nav -Force
    Remove-Item -LiteralPath ($nav + '.bak') -Force
    Add-Content -LiteralPath $out -Value 'DONE'
}
