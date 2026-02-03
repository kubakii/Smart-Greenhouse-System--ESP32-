# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/bot/esp32/esp-idf/components/bootloader/subproject"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/tmp"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/src/bootloader-stamp"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/src"
  "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/bot/esp32/SmartGreenhouse/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
