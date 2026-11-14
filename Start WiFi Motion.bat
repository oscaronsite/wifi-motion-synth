@echo off
rem ------------------------------------------------------------------
rem  WiFi Motion - double-click to start the page.
rem  Uses the Python that came with ESP-IDF if it is installed,
rem  otherwise the normal Python of this computer.
rem  Stop it with Ctrl+C (then Y), or just close this window.
rem ------------------------------------------------------------------
cd /d "%~dp0"
set "IDFPY=C:\Espressif\tools\python\v5.5.6\venv\Scripts\python.exe"
if exist "%IDFPY%" goto idf
where py >nul 2>nul
if not errorlevel 1 goto pylauncher
python csi_viz.py %*
goto end
:idf
"%IDFPY%" csi_viz.py %*
goto end
:pylauncher
py csi_viz.py %*
:end
echo.
echo The page has stopped. Press any key to close this window.
pause >nul
