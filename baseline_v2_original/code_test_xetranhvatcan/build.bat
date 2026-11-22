@echo off
chcp 65001 >nul
echo ========================================================
echo       BUILD DỰ ÁN CODE TEST XE TRÁNH VẬT CẢN (STM32)
echo ========================================================

set "GCC_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "NINJA_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin"
set "CMAKE_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin"

set "PATH=%GCC_PATH%;%NINJA_PATH%;%CMAKE_PATH%;%PATH%"

echo [1/2] Dang cau hinh CMake (Debug)...
cmake --preset Debug
if %ERRORLEVEL% NEQ 0 (
    echo [LOI] Cau hinh CMake that bai!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [2/2] Dang bien dich du an va tao file .hex, .bin, .elf...
cmake --build --preset Debug
if %ERRORLEVEL% NEQ 0 (
    echo [LOI] Bien dich that bai!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo ========================================================
echo  BUILD THANH CONG! File nam tai:
echo    - Hex: build\Debug\code_test_xetranhvatcan.hex
echo    - Bin: build\Debug\code_test_xetranhvatcan.bin
echo    - Elf: build\Debug\code_test_xetranhvatcan.elf
echo ========================================================
pause

