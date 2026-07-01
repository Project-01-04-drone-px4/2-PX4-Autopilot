@echo off
setlocal

set "REPO_WIN=%~dp0"

for /f "tokens=5" %%P in ('netstat -ano ^| findstr ":4560"') do (
    taskkill /PID %%P /F >nul 2>nul
)

for /f "tokens=2 delims=," %%P in ('tasklist /FI "IMAGENAME eq px4.exe" /FO CSV /NH 2^>nul') do (
    taskkill /PID %%~P /F >nul 2>nul
)

call "%REPO_WIN%clean_build_sitl_cygwin.cmd"
if errorlevel 1 exit /b %ERRORLEVEL%

call "%REPO_WIN%run_jmavsim_cygwin.cmd"
