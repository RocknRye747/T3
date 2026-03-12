cmake_minimum_required(VERSION 3.16)
project(reflex_firmware ASM)

# Add the assembly source file
add_executable(reflex_firmware firmware/reflex/reflex.S)

# Note: For ESP32 projects, you would typically integrate with ESP-IDF.
# This might involve including ESP-IDF's CMake modules and setting specific toolchain files.
# Example (uncomment and adapt if using ESP-IDF):
# set(IDF_PATH "$ENV{IDF_PATH}" CACHE PATH "Path to ESP-IDF")
# if(NOT IDF_PATH)
#    message(FATAL_ERROR "IDF_PATH is not set. Please set it to your ESP-IDF installation directory.")
# endif()
# include($ENV{IDF_PATH}/tools/cmake/project.cmake)
