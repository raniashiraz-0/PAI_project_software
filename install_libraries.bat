@echo off
REM ============================================================================
REM  Copies the TFT libraries (MCUFRIEND_kbv, Adafruit GFX, BusIO, TouchScreen)
REM  into your Arduino IDE libraries folder. Double-click this file.
REM  Only needed if you have NOT installed them with the Library Manager already.
REM ============================================================================
setlocal
for /f "usebackq delims=" %%D in (`powershell -NoProfile -Command "[Environment]::GetFolderPath('MyDocuments')"`) do set "DOCS=%%D"
set "DEST=%DOCS%\Arduino\libraries"
echo Installing TFT libraries into:
echo   %DEST%
if not exist "%DEST%" mkdir "%DEST%"
for %%L in (MCUFRIEND_kbv Adafruit_GFX_Library Adafruit_BusIO Adafruit_TouchScreen) do (
    echo   - %%L
    robocopy "%~dp0libraries\%%L" "%DEST%\%%L" /E /NFL /NDL /NJH /NJS /NP >nul
)
echo.
echo Done. Restart the Arduino IDE, then open uno_tft_display\uno_tft_display.ino
pause
