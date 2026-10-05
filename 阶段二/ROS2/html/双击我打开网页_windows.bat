@echo off
chcp 65001 >nul
cd /d "%~dp0"
title 学习日志 - 本地网页

where python >nul 2>nul
if %errorlevel%==0 (
    python serve.py
    goto done
)

where py >nul 2>nul
if %errorlevel%==0 (
    py -3 serve.py
    goto done
)

echo.
echo   ============================================
echo    没有找到 Python，无法启动本地网页服务。
echo.
echo    请到 https://www.python.org/downloads/ 下载安装，
echo    安装时务必勾选 "Add Python to PATH"，
echo    装完再双击一次本文件。
echo   ============================================
echo.

:done
echo.
echo 网页服务已停止。按任意键关闭本窗口。
pause >nul
