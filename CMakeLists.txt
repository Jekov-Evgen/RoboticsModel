cmake_minimum_required(VERSION 3.20)
project(RoboticsModel VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOTICS_BUILD_TESTS "Build the unit tests" ON)
option(ROBOTICS_BUILD_EDITOR "Build the macOS editor (Apple only)" ON)
option(ROBOTICS_BUILD_PYTHON "Build the Python bindings (needs Python and pybind11)" OFF)
option(ROBOTICS_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(ROBOTICS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ---- Shared build settings (interface targets) -------------------------------------------------

add_library(robotics_warnings INTERFACE)
add_library(robotics::warnings ALIAS robotics_warnings)
if(MSVC)
  target_compile_options(robotics_warnings INTERFACE /W4 $<$<BOOL:${ROBOTICS_WARNINGS_AS_ERRORS}>:/WX>)
else()
  target_compile_options(robotics_warnings INTERFACE
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wcast-align -Woverloaded-virtual -Wimplicit-fallthrough
    $<$<BOOL:${ROBOTICS_WARNINGS_AS_ERRORS}>:-Werror>)
endif()

add_library(robotics_sanitizers INTERFACE)
add_library(robotics::sanitizers ALIAS robotics_sanitizers)
if(ROBOTICS_ENABLE_SANITIZERS AND NOT MSVC)
  target_compile_options(robotics_sanitizers INTERFACE
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  target_link_options(robotics_sanitizers INTERFACE -fsanitize=address,undefined)
endif()

# ---- Core library: headless, no UI dependencies ---------------------------------------------------
# Module directories: physics (bodies, collision, solver, actuators), control, io, scenes, sensors,
# editing (assembly edits, gizmo math, part library), replay (recording, plot channels).

file(GLOB ROBOTICS_CORE_SOURCES CONFIGURE_DEPENDS
  src/physics/*.cpp src/control/*.cpp src/io/*.cpp src/scenes/*.cpp src/sensors/*.cpp
  src/editing/*.cpp src/replay/*.cpp)
add_library(robotics ${ROBOTICS_CORE_SOURCES})
add_library(robotics::robotics ALIAS robotics)
target_include_directories(robotics
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE src)
target_compile_features(robotics PUBLIC cxx_std_20)
# PIC lets the static library be linked into a shared module (e.g. future Python bindings).
set_target_properties(robotics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(robotics PRIVATE robotics::warnings PUBLIC robotics::sanitizers)

# ---- Python bindings (pip install .  or  -DROBOTICS_BUILD_PYTHON=ON) ----------------------------------

if(ROBOTICS_BUILD_PYTHON)
  find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(robotics_py MODULE python/src/module.cpp)
  set_target_properties(robotics_py PROPERTIES OUTPUT_NAME _core)
  target_link_libraries(robotics_py PRIVATE robotics)
  install(TARGETS robotics_py LIBRARY DESTINATION robotics_model)
endif()

# ---- Command-line runner ---------------------------------------------------------------------------

add_executable(robotics_cli src/cli/main.cpp)
target_link_libraries(robotics_cli PRIVATE robotics robotics::warnings)

# Physics micro benchmark (steps/s for rover, 6-link arm, 100-box pile, 50-hull piles).
add_executable(robotics_bench tools/physics_bench.cpp)
target_link_libraries(robotics_bench PRIVATE robotics robotics::warnings)

# ---- Tests -------------------------------------------------------------------------------------------

if(ROBOTICS_BUILD_TESTS)
  enable_testing()
  file(GLOB ROBOTICS_TEST_SOURCES CONFIGURE_DEPENDS tests/*.cpp)
  add_executable(robotics_tests ${ROBOTICS_TEST_SOURCES})
  target_compile_definitions(robotics_tests PRIVATE ROBOTICS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(robotics_tests PRIVATE robotics robotics::warnings)
  add_test(NAME physics_and_scenes COMMAND robotics_tests)
  add_test(NAME cli_smoke COMMAND robotics_cli --seconds 0.5)
  add_test(NAME cli_example_scene
    COMMAND robotics_cli --scene ${CMAKE_CURRENT_SOURCE_DIR}/examples/spatial_arm.robot --seconds 0.5)
  add_test(NAME cli_sensor_csv
    COMMAND robotics_cli --sensor-rover-demo --seconds 0.5 --sensor-csv ${CMAKE_CURRENT_BINARY_DIR}/sensors.csv)
endif()

# ---- macOS editor (Objective-C++ UI on top of the core) ------------------------------------------

if(APPLE AND ROBOTICS_BUILD_EDITOR)
  enable_language(OBJCXX)
  set(CMAKE_OBJCXX_STANDARD 20)
  file(GLOB ROBOTICS_EDITOR_SOURCES CONFIGURE_DEPENDS src/mac/*.mm)
  add_executable(robotics_editor MACOSX_BUNDLE ${ROBOTICS_EDITOR_SOURCES})
  target_include_directories(robotics_editor PRIVATE src)
  target_compile_options(robotics_editor PRIVATE -fobjc-arc)
  # Lets Examples > ... (URDF) find examples/ when the app runs from the build tree.
  target_compile_definitions(robotics_editor PRIVATE ROBOTICS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(robotics_editor PRIVATE robotics robotics::warnings
    "-framework Cocoa" "-framework Metal" "-framework MetalKit" "-framework QuartzCore"
    "-framework UniformTypeIdentifiers")

  add_executable(robotics_icon_builder tools/icon_builder.mm)
  target_include_directories(robotics_icon_builder PRIVATE src)
  target_compile_options(robotics_icon_builder PRIVATE -fobjc-arc)
  target_link_libraries(robotics_icon_builder PRIVATE robotics::warnings "-framework Cocoa")

  set(ROBOTICS_ICNS "${CMAKE_CURRENT_BINARY_DIR}/RoboticsModel.icns")
  add_custom_command(OUTPUT "${ROBOTICS_ICNS}"
    COMMAND robotics_icon_builder "${CMAKE_CURRENT_BINARY_DIR}/RoboticsModel.iconset"
    COMMAND iconutil -c icns "${CMAKE_CURRENT_BINARY_DIR}/RoboticsModel.iconset" -o "${ROBOTICS_ICNS}"
    DEPENDS robotics_icon_builder VERBATIM)
  target_sources(robotics_editor PRIVATE "${ROBOTICS_ICNS}")
  set_source_files_properties("${ROBOTICS_ICNS}" PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  set_target_properties(robotics_editor PROPERTIES
    OUTPUT_NAME RoboticsModel
    MACOSX_BUNDLE_BUNDLE_NAME "Robotics Model"
    MACOSX_BUNDLE_GUI_IDENTIFIER "local.roboticsmodel.editor"
    MACOSX_BUNDLE_ICON_FILE "RoboticsModel.icns")
endif()
