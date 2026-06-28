@echo off
setlocal
echo ============================================
echo ESP-IDF Build for ESP32-P4 (Host)
echo ============================================
echo.
if not defined IDF_PATH (
    echo ERROR: IDF_PATH not set.
    echo Please set it first:
    echo   set IDF_PATH=C:\path\to\esp-idf
    echo Or run this script from an ESP-IDF Command Prompt.
    pause
    exit /b 1
)
echo Using ESP-IDF from: %IDF_PATH%
echo.
echo Running idf.py reconfigure...
call idf.py reconfigure
if errorlevel 1 (
    echo ERROR: reconfigure failed
    pause
    exit /b 1
)
echo.
echo Reconfigure complete! Next run:
echo   idf.py build flash monitor
pause
endlocal
