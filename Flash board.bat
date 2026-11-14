@echo off
rem ------------------------------------------------------------------
rem  WiFi Motion - copies app_main.c from this folder into the ESP-IDF
rem  project and flashes the board. Plug the board in with USB first.
rem  Needs ESP-IDF v5.5.6 and the esp-csi project (see README.md).
rem ------------------------------------------------------------------
set "PROJECT=C:\esp-csi\examples\get-started\csi_recv_router"
set "PROFILE=C:\Espressif\tools\Microsoft.v5.5.6.PowerShell_profile.ps1"
if not exist "%PROJECT%\main" goto noproject
if not exist "%PROFILE%" goto noidf
copy /y "%~dp0app_main.c" "%PROJECT%\main\app_main.c" >nul
powershell -NoProfile -ExecutionPolicy Bypass -Command ". '%PROFILE%'; Set-Location '%PROJECT%'; idf.py flash"
echo.
echo Finished. If it ended with "Hard resetting", unplug and replug the board.
echo If it said "Failed to connect": unplug, hold BOOT, plug in, release, run this again.
goto end
:noproject
echo The ESP-IDF project folder was not found: %PROJECT%
echo See "Setting up flashing on a new laptop" in README.md.
goto end
:noidf
echo ESP-IDF v5.5.6 was not found: %PROFILE%
echo See "Setting up flashing on a new laptop" in README.md.
:end
echo.
echo Press any key to close this window.
pause >nul
