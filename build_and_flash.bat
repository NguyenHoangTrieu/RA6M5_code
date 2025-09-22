set PATH=%PATH%;C:\Program Files (x86)\Renesas Electronics\Programming Tools\Renesas Flash Programmer V3.20
rmdir /s /q build
mkdir build
cd build
cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Release ..
ninja
@REM "ShowEmuList" | & "C:\Program Files\SEGGER\JLink_V864a\JLink.exe" | Select-String "Serial number"
rfp-cli -device RA -t jlink -tool jlink:831351518 -auto -erase -program RA6M5_code.hex -verify