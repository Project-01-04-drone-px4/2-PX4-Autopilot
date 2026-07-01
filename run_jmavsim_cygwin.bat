@echo off
setlocal

set "PX4_REPO=%~dp0"
set "CYGWIN_HOME=C:\PX4\toolchain\cygwin64"
set "JAVA_HOME=C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot"
set "ANT_HOME=C:\PX4\toolchain\apache-ant"

"%CYGWIN_HOME%\bin\bash.exe" -lc "export JAVA_HOME='/cygdrive/c/Program Files/Eclipse Adoptium/jdk-17.0.19.10-hotspot'; export ANT_HOME='/cygdrive/c/PX4/toolchain/apache-ant'; export PATH=\"$JAVA_HOME/bin:$ANT_HOME/bin:$PATH\"; cd \"$(cygpath '%PX4_REPO%')\"; make px4_sitl_default jmavsim"

endlocal
