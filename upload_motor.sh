#!/bin/bash
st-flash --connect-under-reset --reset write cmake_build/UVA_SOLAR_CAR/develop/GCC_ARM/Motor/Motor.bin 0x8000000

