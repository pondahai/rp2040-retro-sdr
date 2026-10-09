@echo off
rem Build the board firmware (plain build, links at 0x10000000).
rem
rem   build_uf2.bat [path to arduino-cli]
rem
rem The two -I flags are required: the pure-C sources live in firmware\, and
rem Arduino copies the whole sketch tree into a build directory before
rem compiling, so relative paths cannot escape it. RetroSDR\src\rd_*.c are
rem one-line includes; what actually gets compiled is firmware\*.c.
rem
rem See README.md for the offset build (0x10004000, required by
rem rp2040-retro-loader) -- that is what ships.
rem
rem NOTE: keep this file ASCII. Chinese comments here get mangled by the
rem console code page and cmd tries to execute them as commands.
setlocal
set CLI=%~1
if "%CLI%"=="" set CLI=arduino-cli
set HERE=%~dp0
"%CLI%" compile --fqbn rp2040:rp2040:rpipico ^
  --build-property "compiler.c.extra_flags=-I%HERE%firmware" ^
  --build-property "compiler.cpp.extra_flags=-I%HERE%firmware" ^
  --output-dir "%HERE%build" "%HERE%RetroSDR"
