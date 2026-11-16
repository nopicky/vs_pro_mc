@echo off
REM =============================================
REM TwinCAT ScopeView SVDX -> CSV export script
REM =============================================
REM Usage:
REM   export_scope.bat <svdx_file>
REM   export_scope.bat <svdx_file> <output_csv>
REM
REM Examples:
REM   export_scope.bat ..\tc_measurement\ScopeWizard.svdx
REM   export_scope.bat ..\tc_measurement\ScopeWizard.svdx result.csv
REM =============================================

setlocal

set EXPORT_TOOL_DIR=D:\Program Files (x86)\Beckoff\TwinCAT\Functions\TF3300-Scope-Server
set EXPORT_TOOL=%EXPORT_TOOL_DIR%\TC3ScopeExportTool.exe

if not exist "%EXPORT_TOOL%" (
    echo ERROR: TC3ScopeExportTool.exe not found at:
    echo   %EXPORT_TOOL%
    exit /b 1
)

if "%~1"=="" (
    echo Usage: %~nx0 ^<svdx_file^> [output_csv]
    echo.
    echo   ^<svdx_file^>   Path to .svdx recording file
    echo   [output_csv]   Optional output path (default: ^<svdx_file^>.csv)
    exit /b 1
)

set SVDX_FILE=%~f1
if not exist "%SVDX_FILE%" (
    echo ERROR: SVDX file not found: %SVDX_FILE%
    exit /b 1
)

if "%~2"=="" (
    set CSV_FILE=%~dpn1.csv
) else (
    set CSV_FILE=%~f2
)

echo Source : %SVDX_FILE%
echo Target : %CSV_FILE%
echo.
echo Running export...
echo.

cd /d "%EXPORT_TOOL_DIR%"
"%EXPORT_TOOL%" "svd=%SVDX_FILE%" target="%CSV_FILE%" silent

if %ERRORLEVEL% neq 0 (
    echo.
    echo ERROR: Export failed (code %ERRORLEVEL%)
    exit /b %ERRORLEVEL%
)

echo.
echo Done: %CSV_FILE%
