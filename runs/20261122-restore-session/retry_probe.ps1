$ErrorActionPreference = 'Continue'
$cli = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
$base = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\runs\20261122-restore-session'
$log = Join-Path $base 'retry_probe.log'
Set-Content -LiteralPath $log -Value "START $(Get-Date -Format o)" -Encoding utf8
for ($i = 1; $i -le 4; $i++) {
    $out = Join-Path $base ("retry_probe_$i.txt")
    Add-Content -LiteralPath $log -Value "ATTEMPT $i $(Get-Date -Format o)"
    $p = Start-Process -FilePath $cli -ArgumentList @('-c', 'port=SWD', 'mode=UR', 'freq=100') -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput $out -RedirectStandardError ($out + '.err')
    Add-Content -LiteralPath $log -Value ("EXIT=" + $p.ExitCode)
    if (Test-Path $out) {
        $text = Get-Content -LiteralPath $out -Raw
        Add-Content -LiteralPath $log -Value $text
        if ($text -match 'Device name\s*:\s*STM32') {
            Add-Content -LiteralPath $log -Value 'CONNECT_OK'
            break
        }
    }
    Start-Sleep -Seconds 5
}
Add-Content -LiteralPath $log -Value "END $(Get-Date -Format o)"
