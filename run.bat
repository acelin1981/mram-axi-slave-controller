@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ===== compile =====
if exist simv del /f /q simv
iverilog -g2012 -o simv +incdir+./src -f rtl.f
if errorlevel 1 (
  echo [ERROR] Compile failed.
  exit /b 1
)

REM ===== choose WEIGHT_MRAM_PCT =====
REM Usage: run.bat 0|20|40|60|80|100
set WEIGHT_MRAM_PCT=%1
if "%WEIGHT_MRAM_PCT%"=="" set WEIGHT_MRAM_PCT=100

set OK=0
if "%WEIGHT_MRAM_PCT%"=="0"   set OK=1
if "%WEIGHT_MRAM_PCT%"=="20"  set OK=1
if "%WEIGHT_MRAM_PCT%"=="40"  set OK=1
if "%WEIGHT_MRAM_PCT%"=="60"  set OK=1
if "%WEIGHT_MRAM_PCT%"=="80"  set OK=1
if "%WEIGHT_MRAM_PCT%"=="100" set OK=1

if "%OK%"=="0" (
  echo [ERROR] Invalid WEIGHT_MRAM_PCT: %WEIGHT_MRAM_PCT%
  echo         Allowed: 0 20 40 60 80 100
  echo         Example: run.bat 60
  exit /b 1
)

REM ===== run =====
echo [RUN] WEIGHT_MRAM_PCT=%WEIGHT_MRAM_PCT%
vvp simv +SCALE=256 +WEIGHT_MODE=C +WEIGHT_MRAM_PCT=%WEIGHT_MRAM_PCT%

echo.
echo [DONE] Simulation finished.
endlocal

