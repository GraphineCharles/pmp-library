
project(pmp-library)

cmake_minimum_required(VERSION 3.0.2)

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()

cmake_policy(SET CMP0048 NEW)
if(${CMAKE_VERSION} VERSION_GREATER "3.11.0")
cmake_policy(SET CMP0072 OLD)
endif()

set(PMP_MAJOR_VERSION 1)
set(PMP_MINOR_VERSION 2)
set(PMP_PATCH_VERSION 1)
set(PMP_VERSION ${PMP_MAJOR_VERSION}.${PMP_MINOR_VERSION}.${PMP_PATCH_VERSION})

set(CMAKE_PROJECT_VERSION ${PMP_VERSION})
set(CMAKE_PROJECT_VERSION_MAJOR ${PMP_MAJOR_VERSION})
set(CMAKE_PROJECT_VERSION_MINOR ${PMP_MINOR_VERSION})
set(CMAKE_PROJECT_VERSION_PATCH ${PMP_PATCH_VERSION})

option(PMP_BUILD_APPS     "Build the PMP applications"  ON)
option(PMP_BUILD_EXAMPLES "Build the PMP examples"      ON)
option(PMP_BUILD_TESTS    "Build the PMP test programs" ON)
option(PMP_BUILD_DOCS     "Build the PMP documentation" ON)

# set output paths
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(GLFW_SOURCE_DIR "external/glfw")
set(GLEW_SOURCE_DIR "external/glew")
set(IMGUI_SOURCE_DIR "external/imgui")
set(EIGEN_SOURCE_DIR "external/eigen")
set(RPLY_SOURCE_DIR "external/rply")
set(STBI_SOURCE_DIR "external/stb_image")

# setup STB Image
# (place *before* GLFW since GLFW has an old copy of stb_image_write.h)
include_directories(${STBI_SOURCE_DIR})
add_subdirectory(${STBI_SOURCE_DIR})

# Building only the GLFW lib
set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
set(GLFW_INSTALL OFF CACHE BOOL "")

# setup use of GLFW and GLEW
if (NOT EMSCRIPTEN)
    add_subdirectory(${GLFW_SOURCE_DIR} ${GLEW_SOURCE_DIR})

    include_directories(${GLFW_SOURCE_DIR}/include
                        ${GLFW_SOURCE_DIR}/deps
                        ${GLEW_SOURCE_DIR}/include)

    add_definitions(-DGLEW_STATIC)
    add_library(glew STATIC
                ${GLEW_SOURCE_DIR}/src/glew.c
                ${GLEW_SOURCE_DIR}/include)
    target_link_libraries(glew ${GLFW_LIBRARIES})

else()

    # GLEW and GLFW are built-in
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_GLFW=3")

endif()

# setup IMGUI
include_directories(${IMGUI_SOURCE_DIR})
add_subdirectory(${IMGUI_SOURCE_DIR})

# setup PLY
include_directories(${RPLY_SOURCE_DIR})
add_subdirectory(${RPLY_SOURCE_DIR})

# setup Eigen
include_directories(${EIGEN_SOURCE_DIR})

include(AddFileDependencies)
include_directories(${PROJECT_SOURCE_DIR}/src/)

# setup for code coverage testing
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug" AND CMAKE_COMPILER_IS_GNUCXX AND ENABLE_COVERAGE)
  enable_testing()

  # set compiler flags
  set(CMAKE_CXX_FLAGS "-g -O0 --coverage")

  # find required tools
  find_program(GCOV gcov)
  find_program(LCOV lcov)
  find_program(GENHTML genhtml)

  # add coverage target
  add_custom_target(coverage

    # cleanup old counters
    ${LCOV} --directory . --zerocounters

    # run test command
    COMMAND make test

    # gather data, generate report
    COMMAND ${LCOV} --directory . --capture --output-file coverage.info
    COMMAND ${LCOV} --remove coverage.info '*/external/*' '*/tests/*' '/usr/*' --output-file coverage.info.cleaned
    COMMAND ${GENHTML} --demangle-cpp -o coverage coverage.info.cleaned
    COMMAND ${CMAKE_COMMAND} -E remove coverage.info coverage.info.cleaned

    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
endif()

# set default compiler flags
if((UNIX OR APPLE) AND CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pedantic -Wall -Wextra -Wno-deprecated-copy")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER "7.0.0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-int-in-bool-context")
  endif()
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND NOT EMSCRIPTEN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pedantic -Wall -Wextra -Wno-unused-parameter")
endif()

if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_USE_MATH_DEFINES -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS")
endif()

if (EMSCRIPTEN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -s WASM=1 --no-heap-copy -s USE_WEBGL2=1 -s ALLOW_MEMORY_GROWTH=1 -s MINIFY_HTML=0")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()


# make a release build by default
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified. Using 'Release' as default.")
  set(CMAKE_BUILD_TYPE "Release"
    CACHE STRING "The build type. Possible values: Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()


if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
endif()

# choose size of the scalar type
if(PMP_SCALAR_TYPE EQUAL 64)
  message(STATUS "Using 64-bit scalar type")
  add_definitions(-DPMP_SCALAR_TYPE_64)
endif()

# choose size of the index type
if(PMP_INDEX_TYPE EQUAL 64)
  message(STATUS "Using 64-bit index type")
  add_definitions(-DPMP_INDEX_TYPE_64)
endif()

# setup clang-tidy if program found
include(clang-tidy)


# which directories to process
if (EMSCRIPTEN)
    add_subdirectory(src/pmp)
    if (PMP_BUILD_APPS)
        add_subdirectory(src/apps)
    endif()
else()
    add_subdirectory(src/pmp)
    if (PMP_BUILD_APPS)
        add_subdirectory(src/apps)
    endif()
    if (PMP_BUILD_DOCS)
        add_subdirectory(docs)
    endif()
    if (PMP_BUILD_EXAMPLES)
        add_subdirectory(examples)
    endif()
    if (PMP_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()

set(CPACK_PACKAGE_VERSION ${PMP_VERSION})
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_SOURCE_IGNORE_FILES "/build/;/.git/;~$;${CPACK_SOURCE_IGNORE_FILES}")
set(CPACK_SOURCE_GENERATOR "ZIP")
include(CPack)

if(NOT EMSCRIPTEN)

  # Generate package configuration files
  include(CMakePackageConfigHelpers)

  set(PMP_CONFIG_PATH "lib${LIB_SUFFIX}/cmake/pmp")

  configure_package_config_file(cmake/pmpConfig.cmake.in
    pmpConfig.cmake
    INSTALL_DESTINATION "${PMP_CONFIG_PATH}"
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  install(EXPORT pmpTargets FILE pmpTargets.cmake
    EXPORT_LINK_INTERFACE_LIBRARIES
    DESTINATION "${PMP_CONFIG_PATH}")

  export(TARGETS pmp FILE pmpTargets.cmake)

  write_basic_package_version_file(pmpConfigVersion.cmake
    VERSION ${PMP_VERSION}
    COMPATIBILITY SameMajorVersion)

  install(FILES "${PROJECT_BINARY_DIR}/pmpConfig.cmake"
    "${PROJECT_BINARY_DIR}/pmpConfigVersion.cmake"
    DESTINATION "${PMP_CONFIG_PATH}")

endif()

# add uninstall target if none is defined
if(NOT TARGET uninstall)

  configure_file("${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
    cmake_uninstall.cmake IMMEDIATE @ONLY)

  add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake")

endif()

# add clang-format target if program is found
include(clang-format)
