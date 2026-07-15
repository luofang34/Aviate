cmake_minimum_required(VERSION 3.10)
project(aviate_gz_plugin VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# gz-msgs10's generated `.pb.h` files require matching protobuf
# headers; mixing versions (e.g. anaconda's older protobuf landing on
# the include path before brew's) fires `PROTOBUF_VERSION != ...`
# and the build tanks early. Resolve protobuf via a brew-discovered
# prefix on macOS (overridable via `AVIATE_PROTOBUF_PREFIX`) and via
# the standard system path on Linux. The prefix is placed BEFORE
# SYSTEM so transitive include paths can't shadow it.
set(AVIATE_PROTOBUF_PREFIX "" CACHE PATH
    "Override the protobuf install prefix used by the gz plugin build")
if(APPLE AND NOT AVIATE_PROTOBUF_PREFIX)
    find_program(BREW_EXECUTABLE brew)
    if(BREW_EXECUTABLE)
        execute_process(
            COMMAND ${BREW_EXECUTABLE} --prefix protobuf
            OUTPUT_VARIABLE AVIATE_PROTOBUF_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _brew_protobuf_rc
        )
        if(NOT _brew_protobuf_rc EQUAL 0)
            set(AVIATE_PROTOBUF_PREFIX "")
        endif()
    endif()
endif()
if(AVIATE_PROTOBUF_PREFIX)
    message(STATUS "Using protobuf prefix: ${AVIATE_PROTOBUF_PREFIX}")
    include_directories(BEFORE SYSTEM "${AVIATE_PROTOBUF_PREFIX}/include")
    list(PREPEND CMAKE_PREFIX_PATH "${AVIATE_PROTOBUF_PREFIX}")
endif()

# Find Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

# Some protobuf builds (e.g. Homebrew's) require linking the internal
# Abseil check-op target explicitly; older Abseil (e.g. Ubuntu's
# libabsl-dev) neither defines nor needs it — protobuf pulls Abseil in
# transitively there. Link it only where the target exists.
set(AVIATE_ABSL_CHECK_OP "")
if(TARGET absl::log_internal_check_op)
    set(AVIATE_ABSL_CHECK_OP absl::log_internal_check_op)
endif()

# Get GZ include/lib directories
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Found gz-sim version: ${GZ_SIM_VER}")

#------------------------------------------------------------------------------
# gz-sim System Plugin (loaded by Gazebo)
#------------------------------------------------------------------------------
add_library(AviateGzPlugin SHARED
    AviateGzPlugin.cc
)

target_link_libraries(AviateGzPlugin
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    protobuf::libprotobuf
    ${AVIATE_ABSL_CHECK_OP}
)

target_include_directories(AviateGzPlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    # Rust-owned shm contract (#262): the cbindgen-generated header
    # is the single layout source; never mirror it by hand.
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../contract/include
)

# Install plugin to standard gz-sim plugin path
install(TARGETS AviateGzPlugin
    LIBRARY DESTINATION lib/gz-sim-${GZ_SIM_VER}/plugins
)

#------------------------------------------------------------------------------
# MotorFailurePlugin (PX4-compatible motor failure simulation)
#------------------------------------------------------------------------------
add_library(MotorFailurePlugin SHARED
    MotorFailurePlugin.cc
)

target_link_libraries(MotorFailurePlugin
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-plugin2::register
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    protobuf::libprotobuf
    ${AVIATE_ABSL_CHECK_OP}
)

target_include_directories(MotorFailurePlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS MotorFailurePlugin
    LIBRARY DESTINATION lib/gz-sim-${GZ_SIM_VER}/plugins
)

#------------------------------------------------------------------------------
# C Bridge Library (for Rust FFI)
#------------------------------------------------------------------------------
add_library(aviate_gz_bridge SHARED
    aviate_gz_bridge.cc
)

target_include_directories(aviate_gz_bridge PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against rt for shared memory (shm_open) — Linux only;
# macOS folds POSIX RT into libSystem so the linker can't find -lrt.
if(NOT APPLE)
    target_link_libraries(aviate_gz_bridge rt)
endif()

# Also create static library for easier Rust linking
add_library(aviate_gz_bridge_static STATIC
    aviate_gz_bridge.cc
)

target_include_directories(aviate_gz_bridge_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(NOT APPLE)
    target_link_libraries(aviate_gz_bridge_static rt)
endif()

# Install bridge library and headers
install(TARGETS aviate_gz_bridge aviate_gz_bridge_static
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    aviate_gz_bridge.h
    DESTINATION include/aviate
)

#------------------------------------------------------------------------------
# Summary
#------------------------------------------------------------------------------
message(STATUS "")
message(STATUS "Aviate Gazebo Plugin Configuration:")
message(STATUS "  Plugin:  AviateGzPlugin -> lib/gz-sim-${GZ_SIM_VER}/plugins/")
message(STATUS "  Bridge:  libaviate_gz_bridge.so -> lib/")
message(STATUS "  Header:  aviate_gz_bridge.h -> include/aviate/")
message(STATUS "")
