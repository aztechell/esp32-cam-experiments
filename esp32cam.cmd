@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\esp32cam.ps1" %*
exit /b %ERRORLEVEL%

