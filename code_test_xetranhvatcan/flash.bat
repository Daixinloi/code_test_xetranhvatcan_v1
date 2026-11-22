@echo off
setlocal
cd /d "%~dp0"
echo ========================================================
echo       NAP CODE STM32 QUA ST-LINK
echo ========================================================
set "HEX_FILE=%~dp0build\Debug\code_test_xetranhvatcan.hex"
set "CUBE_CLI=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "STLINK_CLI=C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
if exist "%HEX_FILE%" goto program
call "%~dp0build.bat"
if errorlevel 1 goto failed
if not exist "%HEX_FILE%" goto failed
:program
if not exist "%CUBE_CLI%" goto legacy
echo Dang nap bang STM32CubeProgrammer...
"%CUBE_CLI%" -c port=SWD mode=UR freq=1000 -w "%HEX_FILE%" -v -rst
if not errorlevel 1 goto success
echo Thu lai voi che do NORMAL...
"%CUBE_CLI%" -c port=SWD mode=NORMAL freq=100 -w "%HEX_FILE%" -v -rst
if not errorlevel 1 goto success
:legacy
if not exist "%STLINK_CLI%" goto failed
"%STLINK_CLI%" -c SWD UR -P "%HEX_FILE%" -V -Rst
if not errorlevel 1 goto success
"%STLINK_CLI%" -c SWD HOTPLUG -P "%HEX_FILE%" -V -Rst
if not errorlevel 1 goto success
:failed
echo [LOI] Nap that bai. Kiem tra thong bao ben tren va day SWD/nguon.
pause
exit /b 1
:success
echo ========================================================
echo NAP, XAC MINH VA RESET STM32 THANH CONG!
echo ========================================================
pause
exit /b 0
