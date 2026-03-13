cmake_minimum_required(VERSION 3.16)

# Default target for this repo's hardware-validation flow.
# You can override with: idf.py set-target <chip>
set(IDF_TARGET esp32s3)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(reflex_firmware)
