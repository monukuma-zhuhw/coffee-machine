@echo off
REM One-click launcher for the serial tool.
REM Installs dependencies on first run, then starts the GUI.

setlocal
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python not found in PATH. Install Python 3.8+ and re-run.
    pause
    exit /b 1
)

REM Probe for pyserial; if missing, install requirements.
python -c "import serial, matplotlib" 1>nul 2>nul
if errorlevel 1 (
    echo [INFO] Installing dependencies...
    python -m pip install -r requirements.txt
    if errorlevel 1 (
        echo [ERROR] pip install failed.
        pause
        exit /b 1
    )
)

python serial_tool.py
endlocal
