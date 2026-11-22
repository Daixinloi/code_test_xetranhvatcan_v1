@echo off
chcp 65001 >nul
echo ========================================================
echo       NẠP CODE CHO STM32 QUA MẠCH NẠP ST-LINK
echo ========================================================

set "HEX_FILE=%~dp0build\Debug\code_test_xetranhvatcan.hex"
set "BIN_FILE=%~dp0build\Debug\code_test_xetranhvatcan.bin"

if not exist "%HEX_FILE%" (
    echo [CANH BAO] Khong tim thay file %HEX_FILE%!
    echo Dang tu dong chay build.bat de tao file nap...
    call "%~dp0build.bat"
)

if not exist "%HEX_FILE%" (
    echo [LOI] Van khong tim thay file hex sau khi build. Vui long kiem tra loi bien dich!
    pause
    exit /b 1
)

set "CUBE_CLI=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "STLINK_CLI=C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"

if exist "%CUBE_CLI%" (
    echo Tim thay STM32CubeProgrammer CLI.
    echo Dang nap qua ST-Link (SWD)...
    "%CUBE_CLI%" -c port=SWD mode=UR -w "%HEX_FILE%" -v -rst
    if %ERRORLEVEL% EQU 0 (
        echo.
        echo ========================================================
        echo  NAP CODE VA RESET CHIP THANH CONG QUA ST-LINK!
        echo ========================================================
        pause
        exit /b 0
    ) else (
        echo [THU LAI] Dang thu ket noi mode binh thuong...
        "%CUBE_CLI%" -c port=SWD mode=NORMAL -w "%HEX_FILE%" -v -rst
        if %ERRORLEVEL% EQU 0 (
            echo.
            echo ========================================================
            echo  NAP CODE VA RESET CHIP THANH CONG QUA ST-LINK!
            echo ========================================================
            pause
            exit /b 0
        )
    )
)

if exist "%STLINK_CLI%" (
    echo Tim thay ST-LINK Utility CLI.
    echo Dang nap qua ST-Link (SWD)...
    "%STLINK_CLI%" -c SWD UR -P "%HEX_FILE%" -V -Rst
    if %ERRORLEVEL% EQU 0 (
        echo.
        echo ========================================================
        echo  NAP CODE VA RESET CHIP THANH CONG QUA ST-LINK!
        echo ========================================================
        pause
        exit /b 0
    ) else (
        echo [THU LAI] Dang thu ket noi mode HotPlug...
        "%STLINK_CLI%" -c SWD HOTPLUG -P "%HEX_FILE%" -V -Rst
        if %ERRORLEVEL% EQU 0 (
            echo.
            echo ========================================================
            echo  NAP CODE VA RESET CHIP THANH CONG QUA ST-LINK!
            echo ========================================================
            pause
            exit /b 0
        )
    )
)

echo.
echo [LOI] Khong the nap duoc code vao STM32!
echo Hay kiem tra cac van de sau:
echo   1. Mach nap ST-Link da cam vao may tinh qua USB chua? (Den ST-Link co sang khong?)
echo   2. Day SWD (SWDIO - PA13, SWCLK - PA14, GND, 3.3V) da cam dung vao STM32 chua?
echo   3. Neu dung BluePill, thu nhan giu nut Reset tren mach STM32, bam nap roi tha nut Reset.
pause
exit /b 1

