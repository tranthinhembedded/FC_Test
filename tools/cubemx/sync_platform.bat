@echo off
setlocal

where py >nul 2>nul
if %ERRORLEVEL%==0 (
    py -3 "%~dp0sync_platform.py"
    exit /b %ERRORLEVEL%
)

where python >nul 2>nul
if %ERRORLEVEL%==0 (
    python "%~dp0sync_platform.py"
    exit /b %ERRORLEVEL%
)

echo Python was not found. Install Python 3 or run tools\cubemx\sync_platform.py manually.
exit /b 1
