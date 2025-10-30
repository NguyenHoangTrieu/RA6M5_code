@echo off
set "PATH=%PATH%;C:\Program Files (x86)\Renesas Electronics\Programming Tools\Renesas Flash Programmer V3.21"
set "ARM_TOOLCHAIN_PATH=C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.3 rel1\bin"

if not exist "%ARM_TOOLCHAIN_PATH%\arm-none-eabi-gcc.exe" (
    echo ERROR: Toolchain not found
    pause
    exit /b 1
)

if exist build rmdir /s /q build
mkdir build
cd build

cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc.cmake -DCMAKE_BUILD_TYPE=Release -DARM_TOOLCHAIN_PATH="%ARM_TOOLCHAIN_PATH%" ..
if errorlevel 1 (
    cd ..
    pause
    exit /b 1
)

ninja
if errorlevel 1 (
    cd ..
    pause
    exit /b 1
)

cd ..
rfp-cli -device RA -t jlink -tool jlink:831351518 -auto -erase -program build/Test_Project.srec -verify

if errorlevel 1 (
    pause
    exit /b 1
)

echo Build and Flash completed!
pause
