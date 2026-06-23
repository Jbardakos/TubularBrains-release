@echo off
REM ====================================================================
REM build_win.bat - build TubularBrains.p for LightWave Modeler (Windows)
REM
REM Requirements:
REM   - Visual Studio (MSVC). Run this from a
REM     "x64 Native Tools Command Prompt for VS" so cl.exe is on PATH.
REM   - The LightWave 2024 SDK (proprietary, NOT shipped in this repo).
REM
REM Same C source as macOS - only the build differs. On Windows the host
REM looks for the exported "_mod_descrip" symbol, which is provided by the
REM SDK's servmain.c and exported via serv.def. We also link the SDK's
REM default startup.c / shutdown.c.
REM ====================================================================
setlocal

set PLUGIN=TubularBrains
set SOURCE=TubularBrains.c

REM --- EDIT THIS to your LightWave 2024 SDK folder (the one with include\) ---
set SDK=C:\LightWave3D_2024.2.0\sdk\lwsdk2024.2

if not exist "%SDK%\include\lwserver.h" (
  echo.
  echo X  Cannot find the SDK at: %SDK%
  echo    Edit the SDK=... line near the top of this script to point at your
  echo    LightWave SDK folder ^(the one containing include\lwserver.h^), then re-run.
  exit /b 1
)

echo ==^> Compiling %SOURCE% ...
cl /nologo /O2 /MT /LD /D_MSWIN ^
   /I"%SDK%\include" ^
   "%SOURCE%" ^
   "%SDK%\source\servmain.c" ^
   "%SDK%\source\startup.c" ^
   "%SDK%\source\shutdown.c" ^
   /Fe:%PLUGIN%.p ^
   /link /DEF:"%SDK%\source\serv.def"

if %ERRORLEVEL% NEQ 0 (
  echo.
  echo X  Build failed. Make sure you are in a VS native tools prompt and the
  echo    SDK path above is correct.
  exit /b 1
)

del /q *.obj 2>nul
echo.
echo OK  Built: %CD%\%PLUGIN%.p
echo.
echo Final step (one time, inside Modeler):
echo     Utilities tab  -^>  Plugins  -^>  Add Plugins
echo     Select:  %CD%\%PLUGIN%.p
echo     The tool then appears as 'TubularBrains'.
endlocal
