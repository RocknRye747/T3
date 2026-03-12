cmake_minimum_required(VERSION 3.16)

# Set the ESP-IDF target. Replace esp32 with your target if different.
set(IDF_TARGET esp32)

# Include ESP-IDF's project.cmake file
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(reflex_firmware)
