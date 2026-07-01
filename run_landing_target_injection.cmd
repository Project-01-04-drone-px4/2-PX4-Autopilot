@echo off
setlocal

cd /d "%~dp0"
python Tools\simulation\jmavsim\send_landing_target.py %*
