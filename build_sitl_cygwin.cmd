@echo off
setlocal

set "REPO_WIN=%~dp0"
set "CYGWIN_BASH=C:\cygwin64\bin\bash.exe"
set "ANT_HOME=%~dp0..\tools\apache-ant-1.10.17"
set "JDK_HOME="

if not exist "%CYGWIN_BASH%" (
    echo Cygwin bash not found at %CYGWIN_BASH%
    exit /b 1
)

for /f "delims=" %%J in ('dir /b /ad "C:\Program Files\Eclipse Adoptium\jdk-*" 2^>nul ^| sort /r') do (
    if not defined JDK_HOME set "JDK_HOME=C:\Program Files\Eclipse Adoptium\%%J"
)

if not defined JDK_HOME (
    echo Eclipse Adoptium JDK not found under C:\Program Files\Eclipse Adoptium
    exit /b 1
)

if not exist "%ANT_HOME%\bin\ant.bat" (
    echo Apache Ant not found at %ANT_HOME%
    exit /b 1
)

call :to_cygwin_path "%REPO_WIN%" REPO_CYG
call :to_cygwin_path "%JDK_HOME%" JDK_CYG
call :to_cygwin_path "%ANT_HOME%" ANT_CYG

"%CYGWIN_BASH%" -lc "export PATH=/home/%USERNAME%/.local/bin:/usr/local/bin:/usr/bin:'%JDK_CYG%/bin':'%ANT_CYG%/bin'; export JAVA_HOME='%JDK_CYG%'; export PYTHONUTF8=1; cd '%REPO_CYG%' && if [ ! -f build/px4_sitl_cygwin39/build.ninja ]; then cmake -S . -B build/px4_sitl_cygwin39 -G Ninja -DCONFIG=px4_sitl_default -DPython3_EXECUTABLE=/usr/bin/python3.9 -DPYTHON_EXECUTABLE=/usr/bin/python3.9 -DCMAKE_C_FLAGS='-include asm/socket.h' -DCMAKE_CXX_FLAGS='-include asm/socket.h'; fi && ninja -C build/px4_sitl_cygwin39"

exit /b %ERRORLEVEL%

:to_cygwin_path
setlocal EnableDelayedExpansion
set "WIN_PATH=%~1"
set "DRIVE=!WIN_PATH:~0,1!"
set "REST=!WIN_PATH:~2!"
set "REST=!REST:\=/!"
for %%D in (A B C D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if /I "!DRIVE!"=="%%D" set "DRIVE=%%D"
)
set "DRIVE=!DRIVE:A=a!"
set "DRIVE=!DRIVE:B=b!"
set "DRIVE=!DRIVE:C=c!"
set "DRIVE=!DRIVE:D=d!"
set "DRIVE=!DRIVE:E=e!"
set "DRIVE=!DRIVE:F=f!"
set "DRIVE=!DRIVE:G=g!"
set "DRIVE=!DRIVE:H=h!"
set "DRIVE=!DRIVE:I=i!"
set "DRIVE=!DRIVE:J=j!"
set "DRIVE=!DRIVE:K=k!"
set "DRIVE=!DRIVE:L=l!"
set "DRIVE=!DRIVE:M=m!"
set "DRIVE=!DRIVE:N=n!"
set "DRIVE=!DRIVE:O=o!"
set "DRIVE=!DRIVE:P=p!"
set "DRIVE=!DRIVE:Q=q!"
set "DRIVE=!DRIVE:R=r!"
set "DRIVE=!DRIVE:S=s!"
set "DRIVE=!DRIVE:T=t!"
set "DRIVE=!DRIVE:U=u!"
set "DRIVE=!DRIVE:V=v!"
set "DRIVE=!DRIVE:W=w!"
set "DRIVE=!DRIVE:X=x!"
set "DRIVE=!DRIVE:Y=y!"
set "DRIVE=!DRIVE:Z=z!"
endlocal & set "%~2=/cygdrive/%DRIVE%%REST%"
exit /b 0
