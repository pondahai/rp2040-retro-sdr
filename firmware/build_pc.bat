@echo off
rem Builds and runs the PC test (synthetic ADC blocks through the whole DSP and
rem UI path). Needs gcc on PATH (MinGW-w64 / WinLibs).
rem
rem NOTE: keep this file pure ASCII. cmd.exe parses .bat with the system code
rem page (cp950 here); Chinese in a rem line gets mangled. Same rule as the
rem build scripts in the repo root.
setlocal
pushd "%~dp0"
gcc -std=c11 -O2 -Wall -Wextra -o test_pc.exe test_pc.c sdr.c ui.c spectrum.c fft.c wfall.c font5x7.c keys.c ddc.c jjy.c wav.c preset.c eq.c -lm
if errorlevel 1 goto fail
.\test_pc.exe
if errorlevel 1 goto fail
popd
exit /b 0
:fail
popd
exit /b 1
