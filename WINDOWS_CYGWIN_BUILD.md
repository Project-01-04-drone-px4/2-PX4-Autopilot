# PX4 Windows Cygwin Build Notes

This repository is configured to build PX4 on Windows through Cygwin, without
WSL. The examples below assume the repository is located at:

```text
E:\18-px4\01-gg-fork\2-PX4-Autopilot
```

The Cygwin toolchain is installed at:

```text
C:\PX4\toolchain\cygwin64
```

The jMAVSim Java tools are installed at:

```text
C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot
C:\PX4\toolchain\apache-ant
```

## Start SITL With jMAVSim

The helper script in the repository root starts SITL and jMAVSim with the
correct Cygwin, Java, and Ant environment:

```bat
E:\18-px4\01-gg-fork\2-PX4-Autopilot\run_jmavsim_cygwin.bat
```

Equivalent manual command from PowerShell or `cmd.exe`:

```powershell
C:\PX4\toolchain\cygwin64\bin\bash.exe -lc 'export JAVA_HOME="/cygdrive/c/Program Files/Eclipse Adoptium/jdk-17.0.19.10-hotspot"; export ANT_HOME="/cygdrive/c/PX4/toolchain/apache-ant"; export PATH="$JAVA_HOME/bin:$ANT_HOME/bin:$PATH"; cd /cygdrive/e/18-px4/01-gg-fork/2-PX4-Autopilot; make px4_sitl_default jmavsim'
```

Expected healthy startup signs:

```text
BUILD SUCCESSFUL
INFO  [simulator_mavlink] Simulator connected on TCP port 4560.
INFO  [px4] Startup script returned successfully
```

Typical ports after startup:

```text
TCP 4560   jMAVSim simulator connection
UDP 14580  PX4 onboard MAVLink
UDP 18570  PX4 normal MAVLink
```

## Build a Normal Hardware Firmware

Hardware firmware targets use this general format:

```bash
make <vendor>_<board>_default
```

Common examples:

```bash
make px4_fmu-v5_default
make px4_fmu-v6c_default
make px4_fmu-v6x_default
make holybro_kakutef7_default
```

From Windows, run the build through Cygwin. For example, to build FMUv5:

```powershell
C:\PX4\toolchain\cygwin64\bin\bash.exe -lc 'export PATH="/cygdrive/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin:$PATH"; cd /cygdrive/e/18-px4/01-gg-fork/2-PX4-Autopilot; make px4_fmu-v5_default'
```

The ARM toolchain path is required for hardware firmware builds:

```text
C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin
```

The resulting firmware file is normally written under the matching build
directory. For FMUv5:

```text
build\px4_fmu-v5_default\px4_fmu-v5_default.px4
```

## List Available Default Targets

To list available default board targets:

```powershell
C:\PX4\toolchain\cygwin64\bin\bash.exe -lc 'cd /cygdrive/e/18-px4/01-gg-fork/2-PX4-Autopilot; make list_config_targets | grep _default'
```

## Notes

- `px4_sitl_default jmavsim` is for simulation and uses the native Cygwin
  compiler plus Java/Ant for jMAVSim.
- Hardware firmware such as `px4_fmu-v5_default` requires the ARM
  `arm-none-eabi-*` toolchain in `PATH`.
- This setup intentionally avoids WSL.
