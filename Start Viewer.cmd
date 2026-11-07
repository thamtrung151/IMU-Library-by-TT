@echo off
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 (
    python tools\serve_viewer.py --open
) else (
    py -3 tools\serve_viewer.py --open
)
pause
