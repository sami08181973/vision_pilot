@echo off
cd /d "%~dp0"
powershell -NoProfile -File "%~dp0run_perf_bench.ps1" %*
exit /b %ERRORLEVEL%
