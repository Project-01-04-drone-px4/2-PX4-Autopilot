@echo off
setlocal

cd /d "%~dp0"
python Tools\simulation\jmavsim\check_landing_target_log.py %*
exit /b %ERRORLEVEL%
