@echo off
setlocal

set "REPO_WIN=%~dp0"
set "BUILD_DIR=%REPO_WIN%build\px4_sitl_cygwin39"

cd /d "%REPO_WIN%"

if exist "%BUILD_DIR%" (
    echo Removing %BUILD_DIR%
    rmdir /s /q "%BUILD_DIR%"
    if exist "%BUILD_DIR%" (
        echo Failed to remove %BUILD_DIR%
        exit /b 1
    )
) else (
    echo Build directory does not exist: %BUILD_DIR%
)

call "%REPO_WIN%build_sitl_cygwin.cmd"
exit /b %ERRORLEVEL%
