@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"

:: ===========================================================================
:: build_offset.bat - offset build for rp2040-retro-loader
::
:: NOTE: keep this file pure ASCII. cmd.exe parses .bat with the system code
:: page (cp950 here); UTF-8 Chinese in comments corrupts the surrounding
:: syntax. The Chinese write-up lives in README.md and loader_offset/*.py.
::
:: Difference from build_uf2.bat: the image links at 0x10004000 instead of
:: 0x10000000, leaving the first 16KB to the loader (when started from SD) or
:: to the trampoline (when flashed over USB). DESIGN.md explains why
:: this, not the plain build, is what ships.
::
:: Optional: set SDR_DEFS=-DKBD_SHARED_DATA=1 before running (see docs/PLAN-IQ.md).
::
:: Usage:
::     build_offset.bat [arduino-cli path] [loader repo path]
::
:: Outputs (under build_offset\):
::     RetroSDR.ino.uf2          body only, first 16KB empty - NOT flashable
::                                alone; this is the file for the SD card when
::                                the loader is already on the board
::     RetroSDR_standalone.uf2   trampoline + body - flashable over USB
:: ===========================================================================

set "HERE=%~dp0"
set "CLI=%~1"
if "%CLI%"=="" set "CLI=arduino-cli"
set "LOADER=%~2"
if "%LOADER%"=="" set "LOADER=%HERE%..\rp2040-retro-loader"
set "OUT_DIR=%HERE%build_offset"
set "OFFSET_LD=%HERE%loader_offset\memmap_app_arduino.ld"

echo.
echo [1/3] Generating offset linker script...
python "%HERE%loader_offset\gen_app_ld.py"
if %errorlevel% neq 0 ( echo [ERROR] gen_app_ld.py failed. & exit /b 1 )

echo.
echo [2/3] Compiling sketch (OFFSET, link at 0x10004000)...
:: recipe.hooks.linking.prelink.1.pattern
::     arduino-pico generates its linker script at build time: simplesub.py
::     substitutes __FLASH_LENGTH__ etc. into lib/rp2040/memmap_default.ld and
::     writes {build.path}/memmap_default.ld, which the link recipe then reads
::     by that hardcoded name. So the way to swap linker scripts is NOT to add
::     -Wl,--script (it would fight the hardcoded one) but to repoint this
::     hook's --input at our offset template. Every other argument is copied
::     verbatim from platform.txt.
::
:: The two -I flags are the same ones build_uf2.bat needs: the pure-C sources
:: live in firmware\ and Arduino copies the sketch tree before compiling.
"%CLI%" compile --fqbn rp2040:rp2040:rpipico ^
  --build-property "compiler.c.extra_flags=-I%HERE%firmware" ^
  --build-property "compiler.cpp.extra_flags=-I%HERE%firmware %SDR_DEFS%" ^
  --build-property "recipe.hooks.linking.prelink.1.pattern=\"{runtime.tools.pqt-python3.path}/python3\" -I \"{runtime.platform.path}/tools/simplesub.py\" --input \"%OFFSET_LD%\" --out \"{build.path}/memmap_default.ld\" --sub __FLASH_LENGTH__ {build.flash_length} --sub __EEPROM_START__ {build.eeprom_start} --sub __FS_START__ {build.fs_start} --sub __FS_END__ {build.fs_end} --sub __RAM_LENGTH__ {build.ram_length} --sub __PSRAM_LENGTH__ {build.psram_length}" ^
  --output-dir "%OUT_DIR%" "%HERE%RetroSDR"
if %errorlevel% neq 0 ( echo [ERROR] Arduino build failed. & exit /b 1 )

echo.
echo [3/3] Checking flash layout...
python "%HERE%loader_offset\check_flash_layout.py" "%OUT_DIR%\RetroSDR.ino.uf2"
if %errorlevel% neq 0 ( echo [ERROR] Layout check failed - do not flash this file. & exit /b 1 )

rem The loader derives the cover art name by chopping ".uf2" off the uf2 name
rem and appending ".RAW" (loader/thumb.c). So every uf2 that can end up in the
rem SD card root needs its own matching .RAW - including the standalone one,
rem which the menu will list even though it is meant for USB flashing.
if exist "%HERE%assets\RetroSDR.ino.RAW" (
    copy /y "%HERE%assets\RetroSDR.ino.RAW" "%OUT_DIR%\RetroSDR.ino.RAW" >nul
    copy /y "%HERE%assets\RetroSDR.ino.RAW" "%OUT_DIR%\RetroSDR_standalone.RAW" >nul
    echo Cover art copied for both uf2 names ^(loader menu^).
) else (
    echo [note] assets\RetroSDR.ino.RAW missing - cover art is milestone M5.
)

if not exist "%LOADER%\build\trampoline.uf2" (
    echo.
    echo [note] "%LOADER%\build\trampoline.uf2" not found, skipping the merge.
    echo        RetroSDR.ino.uf2 is still valid for a board that already has
    echo        the loader in its first 16KB.
    exit /b 0
)

echo.
echo Merging with trampoline...
python "%LOADER%\tools\merge_uf2.py" "%LOADER%\build\trampoline.uf2" ^
    "%OUT_DIR%\RetroSDR.ino.uf2" -o "%OUT_DIR%\RetroSDR_standalone.uf2"
if %errorlevel% neq 0 ( echo [ERROR] merge_uf2.py failed. & exit /b 1 )

echo.
echo SUCCESS
echo   %OUT_DIR%\RetroSDR.ino.uf2          for the SD card (loader on board)
echo   %OUT_DIR%\RetroSDR_standalone.uf2   flashable over USB
echo   %OUT_DIR%\RetroSDR.ino.RAW          cover art for RetroSDR.ino.uf2
echo   %OUT_DIR%\RetroSDR_standalone.RAW   cover art for the standalone uf2
echo.
echo   Name rule: the loader looks for ^<uf2 name minus .uf2^>.RAW, so each uf2
echo   needs the .RAW that matches its own name. Both are in the SD card root.
