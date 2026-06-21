@echo off
setlocal
cd /d "d:\desktopproject\Servo_host"
echo Setting up ESP-IDF environment...
call C:sp5.5.4sp-idfxport.bat
if errorlevel 1 (
  echo ERROR: Failed to set up ESP-IDF environment
  pause
  exit /b 1
)
echo.
echo Running idf.py reconfigure...
idf.py reconfigure
if errorlevel 1 (
  echo ERROR: reconfigure failed
  pause
  exit /b 1
)
echo.
echo Reconfigure complete! Now run idf.py build
pause
endlocal
