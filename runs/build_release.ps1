$ErrorActionPreference = 'Continue'
$proj = 'C:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\code_test_xetranhvatcan'
$log  = 'C:\Users\daitr\Downloads\code_test_xetranhvatcan_v1\runs\build_release.log'
$env:PATH = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin;' +
    'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin;' +
    'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin;' + $env:PATH
Set-Location $proj
Remove-Item -LiteralPath $log -ErrorAction SilentlyContinue
"=== CONFIGURE ===" | Set-Content -LiteralPath $log
& cmake --preset Release *>> $log
"=== BUILD ===" | Add-Content -LiteralPath $log
& cmake --build --preset Release *>> $log
$code = $LASTEXITCODE
"=== EXIT $code ===" | Add-Content -LiteralPath $log
$hex = Join-Path $proj 'build\Release\code_test_xetranhvatcan.hex'
if (Test-Path $hex) {
    "=== SHA256 ===" | Add-Content -LiteralPath $log
    (Get-FileHash $hex -Algorithm SHA256).Hash | Add-Content -LiteralPath $log
    "=== SIZE ===" | Add-Content -LiteralPath $log
    (Get-Item $hex).Length | Add-Content -LiteralPath $log
}
"=== DONE ===" | Add-Content -LiteralPath $log
