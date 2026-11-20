@echo off
chcp 65001 >nul
title HE THONG DIEU KHIEN XE AGV (PID + RFID 3 THE)
cd /d "%~dp0"

echo ============================================================================
echo    KHOI DONG XE AGV: CHAY BAM LINE PID + DOC THE RFID (DUNG 10S - DU 3 THE)
echo ============================================================================
echo.

if exist "C:\Users\cheesecong\AppData\Local\Programs\Python\Python310\python.exe" (
    "C:\Users\cheesecong\AppData\Local\Programs\Python\Python310\python.exe" "%~dp0main_AGV.py" %*
) else (
    python "%~dp0main_AGV.py" %*
)

echo.
echo ============================================================================
echo    Da ket thuc phien chay AGV. Nhan phim bat ky de dong cua so...
echo ============================================================================
pause >nul
