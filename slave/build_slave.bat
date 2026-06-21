@echo off
echo Setting up ESP-IDF environment...
call C:\esp\v5.5.4\esp-idf\export.bat
if errorlevel 1 (
  echo ERROR: Failed to set up ESP-IDF environment
  pause
  exit /b 1
)
echo.
echo ============================================
echo Building C5 SLAVE firmware
echo ============================================
cd /d "d:\desktop\Eproject\Servo_host\slave"
idf.py set-target esp32c5
idf.py reconfigure
idf.py build
if errorlevel 1 (
  echo ERROR: Build failed
  pause
  exit /b 1
)
echo.
echo ============================================
echo C5 Slave firmware build complete!
echo Flash with: idf.py flash
echo ============================================
pause
