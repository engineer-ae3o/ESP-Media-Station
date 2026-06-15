# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_media_station)

add_custom_target(cppcheck
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/.cppcheck
    COMMAND ${CPPCHECK} --enable=all
        --inline-suppr
        --cppcheck-build-dir=${CMAKE_BINARY_DIR}/.cppcheck
        --suppressions-list=${CMAKE_SOURCE_DIR}/cppcheck_suppressions.txt
        --error-exitcode=1
        --check-level=exhaustive
        -I ${CMAKE_SOURCE_DIR}/components/drivers/inc/
        -I ${CMAKE_SOURCE_DIR}/components/test/inc/
        ${CMAKE_SOURCE_DIR}/main/main.cpp
        ${CMAKE_SOURCE_DIR}/components/drivers/src/
        ${CMAKE_SOURCE_DIR}/components/app/src/
    COMMENT "Running static analysis..."
)
