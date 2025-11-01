@echo off
setlocal

set PROJECT_NAME=physics_engine
set BUILD_DIR=build

if "%1"=="" (
    echo Usage:
    echo   run.bat -build   ^(configure & build then run^)
    echo   run.bat -run     ^(run existing build^)
    echo   run.bat -clean   ^(delete build folder^)
    exit /b 1
)

if "%1"=="-clean" (
    echo Removing %BUILD_DIR%...
    if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
    echo Done.
    exit /b 0
)

if "%1"=="-build" (
    if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
    pushd "%BUILD_DIR%"
    cmake -G "MinGW Makefiles" ..
    cmake --build . -- -j
    popd
    echo Running...
    "%BUILD_DIR%\%PROJECT_NAME%.exe"
    exit /b 0
)

if "%1"=="-run" (
    if exist "%BUILD_DIR%\%PROJECT_NAME%.exe" (
        "%BUILD_DIR%\%PROJECT_NAME%.exe"
    ) else (
        echo Build not found. Run with -build first.
    )
    exit /b 0
)

echo Unknown option: %1
exit /b 1
