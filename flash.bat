@echo off
setlocal
set "CLT=E:\STM32CubeCLT_1.18.0"
set "PROG=%CLT%\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "ELF=%~dp0build\Debug\CRTC_CAR_DEMO.elf"
set "LOG=%TEMP%\crtc_flash.log"

if not exist "%ELF%" (
    echo [ERROR] ELF not found: %ELF%
    echo         Run: cmake --build build/Debug
    pause
    exit /b 1
)

echo ===============================================
echo  CRTC_CAR_DEMO  ST-LINK Flash
echo  ELF: %ELF%
echo ===============================================
echo.

"%PROG%" -c port=SWD mode=UR -w "%ELF%" -v -rst > "%LOG%" 2>&1
type "%LOG%"

findstr /C:"File download complete" "%LOG%" >nul
if %errorlevel%==0 (
    echo.
    echo [OK] Flash and verify succeeded. Target reset and running.
) else (
    echo.
    echo [FAILED] Could not program the target.
    echo   Check: target powered with 3.3V / GND connected / SWDIO+SWCLK wiring.
    echo   A healthy target reports "Voltage : 3.2V" or so, not 0.00V / 1.3V.
)
echo.
pause
