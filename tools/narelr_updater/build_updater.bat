@echo off
:: build_updater.bat - build the LR firmware (env BOARD_NARELRUSERCUSTOM) and pack
:: it with esptool into <repo>\NareLR<YYMMDD>_updater.exe.
:: Needs PlatformIO (pio) and Python 3 with pyserial and pyinstaller.
setlocal
cd /d "%~dp0\..\.."

pio run -e BOARD_NARELRUSERCUSTOM || goto :fail

for /f %%v in ('python scripts\narelr_version.py --name') do set VER=%%v
if "%VER%"=="" goto :fail

set ESPTOOL=%USERPROFILE%\.platformio\packages\tool-esptoolpy
set WORK=%TEMP%\narelr_updater_build
if exist "%WORK%" rmdir /s /q "%WORK%"
mkdir "%WORK%"
python -c "import sys,io;s=io.open(r'tools\narelr_updater\narelr_updater.py',encoding='utf-8').read().replace('@VERSION@',sys.argv[1]);io.open(r'%WORK%\narelr_updater.py','w',encoding='utf-8').write(s)" %VER% || goto :fail
copy /y ".pio\build\BOARD_NARELRUSERCUSTOM\firmware.bin" "%WORK%\firmware.bin" >nul

python -m PyInstaller --noconfirm --onefile --console --name "%VER%_updater" ^
  --distpath . --workpath "%WORK%\build" --specpath "%WORK%" ^
  --paths "%ESPTOOL%" --paths "%ESPTOOL%\_contrib" ^
  --hidden-import intelhex ^
  --add-data "%ESPTOOL%\esptool\targets\stub_flasher;esptool\targets\stub_flasher" ^
  --add-data "%WORK%\firmware.bin;." "%WORK%\narelr_updater.py" || goto :fail

echo.
echo Built %VER%_updater.exe
exit /b 0

:fail
echo Build failed.
exit /b 1
