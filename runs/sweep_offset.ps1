$ErrorActionPreference = 'Stop'
$nav = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan\Src\navigation.c'
$proj = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan'
$orig = Get-Content -LiteralPath $nav -Raw
$results = @()
foreach ($mm in @(20, 25, 30, 35, 40, 45, 50, 60)) {
    $patched = $orig -replace '(?m)^#define FIXED_LINE_OFFSET_MM [0-9.]+f$', "#define FIXED_LINE_OFFSET_MM $mm.0f"
    Set-Content -LiteralPath $nav -Value $patched -NoNewline
    Push-Location $proj
    gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_sweep.exe 2>$null
    if ($LASTEXITCODE -ne 0) { Pop-Location; $results += "offset=$mm COMPILE_FAIL"; continue }
    $nom = & '.\build\sim_sweep.exe' trace nominal 2>$null | Select-Object -Last 1
    $t9 = & '.\build\sim_sweep.exe' trace t9 2>$null | Select-Object -Last 1
    Pop-Location
    $results += "offset=$mm | NOM=$nom | T9=$t9"
}
Set-Content -LiteralPath $nav -Value $orig -NoNewline
$results | ForEach-Object { $_ }
