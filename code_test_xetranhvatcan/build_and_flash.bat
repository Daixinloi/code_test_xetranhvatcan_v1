@echo off
cd /d "%~dp0"
chcp 65001 >nul
echo ========================================================
echo       BUILD VA NAP CODE STM32 QUA ST-LINK (1 CLICK)
echo ========================================================

set "GCC_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
set "NINJA_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.ninja.win32_1.1.200.202606260906\tools\bin"
set "CMAKE_PATH=C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cmake.win32_1.1.200.202605190741\tools\bin"

set "PATH=%GCC_PATH%;%NINJA_PATH%;%CMAKE_PATH%;%PATH%"

echo [1/3] Dang cau hinh du an... 
cmake --fresh --preset Debug
if %ERRORLEVEL% NEQ 0 (
    echo [LOI] Cau hinh CMake that bai!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [2/3] Dang bien dich code...
cmake --build --preset Debug
if %ERRORLEVEL% NEQ 0 (
    echo [LOI] Bien dich that bai!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [3/3] Dang nap code vao STM32 qua ST-Link...
call "%~dp0flash.bat"

