cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_media_station)

find_program(CPPCHECK cppcheck)
if (CPPCHECK)
    add_custom_target(cppcheck
        COMMAND mkdir -p ${CMAKE_BINARY_DIR}/.cppcheck
        COMMAND ${CPPCHECK} --enable=all
            --inline-suppr
            --cppcheck-build-dir=${CMAKE_BINARY_DIR}/.cppcheck
            --suppressions-list=${CMAKE_SOURCE_DIR}/cppcheck_suppressions.txt
            --error-exitcode=1
            --check-level=exhaustive
            --project=${CMAKE_BINARY_DIR}/compile_commands.json
            --file-filter=${CMAKE_SOURCE_DIR}/main/*
            --file-filter=${CMAKE_SOURCE_DIR}/components/drivers/**
            --file-filter=${CMAKE_SOURCE_DIR}/components/app/**
            --output-file=${CMAKE_BINARY_DIR}/cppcheck_report.txt
            --checkers-report=${CMAKE_BINARY_DIR}/cppcheck_checks.txt
        COMMENT "Running static analysis..."
    )
else()
    message(WARNING "cppcheck not found")
endif()
