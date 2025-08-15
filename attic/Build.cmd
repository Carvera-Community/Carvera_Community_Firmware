@cmd.exe /K "%~dp0\gcc-arm-none-eabi\bin\buildenv.cmd && make all AXIS=5 PAXIS=3 CNC=1 && copy /Y LPC1768\main.bin %USERPROFILE%\Desktop\firmware.bin"
