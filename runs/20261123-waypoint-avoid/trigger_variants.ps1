# Compare three candidate trigger sources for the fixed-line detour.
#   C (current)  : centre CLEAR  (>= 300) AND a diagonal close (<= 100)
#   A (proposal) : centre CLOSE  (<= 300) AND a diagonal close (<= 100)
#   B (diag only): a diagonal close (<= 100), no centre condition
# Both sim cases must be measured: nominal must stay 14.37 s (no false trigger)
# and t9 must finish without contact.
$ErrorActionPreference = 'Stop'
$nav  = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan\Src\navigation.c'
$proj = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan'
$out  = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\runs\20261123-waypoint-avoid\trigger_variants.txt'

Copy-Item -LiteralPath $nav -Destination ($nav + '.bak') -Force
$orig = Get-Content -LiteralPath $nav -Raw
$res = @()
try {
    $variants = @(
        @{ name = 'C current  centre>=300 AND diag<=100'; from = 'front[1].mm >= FIXED_LINE_CENTRE_MM'; to = 'front[1].mm >= FIXED_LINE_CENTRE_MM' },
        @{ name = 'A proposal centre<=300 AND diag<=100'; from = 'front[1].mm >= FIXED_LINE_CENTRE_MM'; to = 'front[1].mm <= FIXED_LINE_CENTRE_MM' },
        @{ name = 'B diagonal   diag<=100 only';          from = 'front[1].mm >= FIXED_LINE_CENTRE_MM'; to = '1' }
    )
    foreach ($v in $variants) {
        Set-Content -LiteralPath $nav -Value $orig.Replace($v.from, $v.to) -NoNewline
        Push-Location $proj
        gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_var.exe 2>&1 | Out-Null
        $b = $LASTEXITCODE
        $nom = (& .\build\sim_var.exe trace nominal 2>&1 | Select-Object -Last 1)
        $t9  = (& .\build\sim_var.exe trace t9 2>&1 | Select-Object -Last 1)
        Pop-Location
        $res += "$($v.name) | build=$b | NOM=$nom | T9=$t9"
    }
} finally {
    Copy-Item -LiteralPath ($nav + '.bak') -Destination $nav -Force
    Remove-Item -LiteralPath ($nav + '.bak') -Force
}
Set-Content -LiteralPath $out -Value $res -Encoding utf8
Get-Content -LiteralPath $out
