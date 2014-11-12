cmake_minimum_required(VERSION 2.6)

if (NOT DEFINED UTPP_DIR)
    set(UTPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../unittestpp)
endif()

# Additional includes
include_directories(../../include)

# Common source files for all UTs
file(GLOB SOURCES "../../src/*.cpp")
file(GLOB FILES "../../include/*.h")
add_library(ut_lib STATIC ${SOURCES})

# Act as VSM
set(CMAKE_MODULE_PATH "${VSM_SDK_DIR}/share/cmake")
include(ugcs/vsm)

set(EXT_LIB ut_lib ${VSM_LIBS})
add_custom_target(resources COMMENT "Copying resources..."
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources
  ${CMAKE_BINARY_DIR}/resources)
add_dependencies(ut_lib resources)

include(ugcs/ut)
