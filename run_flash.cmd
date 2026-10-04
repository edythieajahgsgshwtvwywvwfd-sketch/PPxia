@echo off
"C:\Users\32455\Desktop\daplink\dap\openocd-20240916\OpenOCD-20240916-0.12.0\bin\openocd.exe" ^
  -f "C:\Users\32455\Desktop\daplink\dap\daplink.cfg" ^
  -c init ^
  -c "reset halt" ^
  -c "program build/Debug/water.elf verify reset exit"
