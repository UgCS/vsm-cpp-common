cmake_minimum_required(VERSION 2.6)

if (NOT DEFINED VSM_SDK_DIR)
    set(VSM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../vsm-cpp-sdk)
endif()

if (NOT DEFINED UTPP_DIR)
    set(UTPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../unittest++)
endif()

# Additional includes
include_directories(../../include)

# Additional source files
file(GLOB SOURCES "../../src/*.cpp")
file(GLOB FILES "../../include/*.h")
set(ADDITIONAL_SRC ${SOURCES} ${FILES})

include("${VSM_SDK_DIR}/make/ut.cmake")
