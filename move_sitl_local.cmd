@echo off
setlocal

cd /d "%~dp0"
python Tools\simulation\jmavsim\move_sitl_local.py %*
exit /b %ERRORLEVEL%
