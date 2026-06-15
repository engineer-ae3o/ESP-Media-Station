cmake_minimum_required(VERSION 3.22)

list(APPEND EXTRA_COMPONENT_DIRS "../components/app")
list(APPEND EXTRA_COMPONENT_DIRS "../components/drivers")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tests)
