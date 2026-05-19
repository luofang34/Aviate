cmake_minimum_required(VERSION 3.10)
project(aviate_gz_plugin VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force Homebrew's protobuf to win the include race against any other
# protobuf in the system path (anaconda, /usr/local, ...). gz-msgs10
# is generated against /opt/homebrew's protobuf 6.33.1; using anyone
# else's headers fires `PROTOBUF_VERSION != 6033001` and the build
# tanks early. The `BEFORE SYSTEM` placement makes this the first
# include path scanned even when anaconda is in PATH.
if(APPLE)
    include_directories(BEFORE SYSTEM /opt/homebrew/include)
endif()

# Find Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED COMPONENTS log_internal_check_op)

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
    absl::log_internal_check_op
)

target_include_directories(AviateGzPlugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
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
    absl::log_internal_check_op
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
