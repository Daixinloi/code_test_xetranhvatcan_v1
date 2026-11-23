$ErrorActionPreference = 'Continue'
$log = 'C:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\runs\tests.log'
Set-Location 'C:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan'
Remove-Item -LiteralPath $log -ErrorAction SilentlyContinue
& powershell -NoProfile -ExecutionPolicy Bypass -File 'Tests\run_checks.ps1' *> $log
"EXIT=$LASTEXITCODE" | Add-Content -LiteralPath $log
