@echo off
REM Deploy completo - ESP32 Multi-Sensor
REM Uso: deploy_all.bat [COM3]
REM Si no se especifica puerto, auto-detecta el primero disponible

echo ==========================================
echo   DEPLOY COMPLETO - ESP32 Multi-Sensor
echo ==========================================

python deploy_all.py %*
if errorlevel 1 (
    echo.
    echo ERROR: Fallo el deploy. Verifica que:
    echo   - PlatformIO este instalado (pip install platformio)
    echo   - pyserial este instalado (pip install pyserial)
    echo   - El ESP32 este conectado via USB
    echo.
    pause
    exit /b 1
)

echo.
pause
