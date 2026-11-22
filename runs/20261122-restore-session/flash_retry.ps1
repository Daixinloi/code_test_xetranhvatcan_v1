$ErrorActionPreference = 'Continue'
$base = 'c:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\runs\20261122-restore-session'
$cli = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
$hex = Join-Path $base 'firmware\build\Release\code_test_xetranhvatcan.hex'
$log = Join-Path $base 'flash_retry.log'
Set-Content -LiteralPath $log -Value ("START " + (Get-Date -Format o)) -Encoding utf8
for ($i = 1; $i -le 6; $i++) {
    $out = Join-Path $base ("flash_try_$i.txt")
    $p = Start-Process -FilePath $cli -ArgumentList @('-c', 'port=SWD', 'mode=UR', 'freq=100', '-w', $hex, '-v', '-rst') -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput $out -RedirectStandardError ($out + '.err')
    $text = ''
    if (Test-Path $out) { $text = Get-Content -LiteralPath $out -Raw }
    Add-Content -LiteralPath $log -Value ("ATTEMPT $i exit=" + $p.ExitCode + " " + (Get-Date -Format o))
    if ($text -match 'File download complete|Download complete|verified successfully|Flash memory programmed') {
        Add-Content -LiteralPath $log -Value 'FLASH_RESULT=OK'
        break
    }
    if ($text -match 'DEV_USB_COMM_ERR|DEV_CONNECT_ERR') {
        Add-Content -LiteralPath $log -Value 'FLASH_RESULT=PROBE_ERROR'
        Start-Sleep -Seconds 10
        continue
    }
    Add-Content -LiteralPath $log -Value ('FLASH_RESULT=OTHER ' + ($text -replace "`r?`n", ' '))
    Start-Sleep -Seconds 5
}
Add-Content -LiteralPath $log -Value ("END " + (Get-Date -Format o))
