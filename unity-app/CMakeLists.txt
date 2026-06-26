cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(EXTRA_COMPONENT_DIRS "../components")
set(TEST_COMPONENTS "drivers app")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_app)
