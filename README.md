# SX127x library for Raspberry Pi PICO (DRAFT VERSION)

Interface library designed for RP2040 microcontroller to support SX127x LoRa modules

## Supported features

    - Transmitting data in non-blocking mode (not with interrupt)
    - Receiving data in non-blocking mode (not with interrupt)
    - Wide specter of module configurations

## How to use library
### Dependences
1. CMake
2. ARM toolchain
3. pico-sdk

### How to compile this library
At first you need to configure pico-sdk for this please see`
https://github.com/raspberrypi/pico-sdk

**The $PICO_SDK_PATH env variable is mandatory to point to the pico-sdk path**

```sh
cd <path_to_project>
cp <path_to_pico_sdk>/external/pico_sdk_import.cmake .
mkdir build
cd build
cmake ..
make
```
After successful compilation you will have **libsx1278.a** library file under *build/lib* directory
You can link this library with your own project and use with header files which are placed under *include* directory

If you get some assembly files compilation error please check the cc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib tools to be installed properly
