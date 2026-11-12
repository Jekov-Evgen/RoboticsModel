# Robotics Model — plain Makefile build (macOS / Apple Clang; core, CLI and tests build on any C++20 compiler).
# CMakeLists.txt builds the same targets; keep both in sync when adding directories.

CXX      ?= clang++
CPPFLAGS  = -Iinclude
WARNINGS  = -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wcast-align -Woverloaded-virtual -Wimplicit-fallthrough
CXXFLAGS  = -std=c++20 -O2 $(WARNINGS)

# Headless core library sources: every .cpp under these module directories.
CORE_DIRS    = src/physics src/control src/io src/scenes src/sensors src/editing src/replay src/kinematics
CORE         = $(foreach dir,$(CORE_DIRS),$(wildcard $(dir)/*.cpp))
CORE_HEADERS = $(wildcard include/robotics/*.hpp include/robotics/*/*.hpp) \
               $(foreach dir,$(CORE_DIRS),$(wildcard $(dir)/*.hpp))
TESTS        = $(wildcard tests/*.cpp)
TEST_HEADERS = $(wildcard tests/*.hpp)
TEST_DEFINES = -DROBOTICS_SOURCE_DIR='"$(CURDIR)"'

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
EDITOR = build/robotics_editor build/RoboticsModel.app/Contents/Info.plist
endif

.PHONY: all test test-ubsan run format clean
all: build/robotics_cli build/robotics_tests $(EDITOR)

build:
	mkdir -p build

build/robotics_cli: src/cli/main.cpp $(CORE) $(CORE_HEADERS) | build
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(CORE) src/cli/main.cpp -o $@

build/robotics_tests: $(TESTS) $(TEST_HEADERS) $(CORE) $(CORE_HEADERS) | build
	$(CXX) $(CPPFLAGS) $(TEST_DEFINES) $(CXXFLAGS) $(CORE) $(TESTS) -o $@

# macOS editor: Objective-C++ UI on top of the core (src/mac only depends on include/robotics).
EDITOR_SRC     = $(wildcard src/mac/*.mm)
EDITOR_HEADERS = $(wildcard src/mac/*.h)
FRAMEWORKS     = -framework Cocoa -framework Metal -framework MetalKit -framework QuartzCore -framework UniformTypeIdentifiers
build/robotics_editor: $(EDITOR_SRC) $(EDITOR_HEADERS) $(CORE) $(CORE_HEADERS) | build
	$(CXX) $(CPPFLAGS) -Isrc $(TEST_DEFINES) $(CXXFLAGS) -fobjc-arc $(CORE) $(EDITOR_SRC) $(FRAMEWORKS) -o $@

build/icon_builder: tools/icon_builder.mm src/mac/app_icon.h | build
	$(CXX) $(CPPFLAGS) -Isrc $(CXXFLAGS) -fobjc-arc tools/icon_builder.mm -framework Cocoa -o $@

build/RoboticsModel.icns: build/icon_builder
	./build/icon_builder build/RoboticsModel.iconset
	iconutil -c icns build/RoboticsModel.iconset -o $@

build/RoboticsModel.app/Contents/Info.plist: build/robotics_editor build/RoboticsModel.icns assets/Info.plist
	mkdir -p build/RoboticsModel.app/Contents/MacOS build/RoboticsModel.app/Contents/Resources
	cp build/robotics_editor build/RoboticsModel.app/Contents/MacOS/RoboticsModel
	cp build/RoboticsModel.icns build/RoboticsModel.app/Contents/Resources/RoboticsModel.icns
	cp assets/Info.plist $@

test: build/robotics_tests
	./build/robotics_tests

build/robotics_tests_ubsan: $(TESTS) $(TEST_HEADERS) $(CORE) $(CORE_HEADERS) | build
	$(CXX) $(CPPFLAGS) $(TEST_DEFINES) -std=c++20 -g -O1 $(WARNINGS) -fsanitize=undefined -fno-sanitize-recover=all $(CORE) $(TESTS) -o $@

test-ubsan: build/robotics_tests_ubsan
	./build/robotics_tests_ubsan

run: build/robotics_editor
	./build/robotics_editor

# Reformat all sources with the repository .clang-format.
FORMAT_FILES = $(shell find include src tests tools -name '*.hpp' -o -name '*.cpp' -o -name '*.h' -o -name '*.mm')
format:
	xcrun clang-format -i $(FORMAT_FILES)

clean:
	$(RM) build/robotics_cli build/robotics_tests build/robotics_tests_ubsan build/robotics_editor build/icon_builder
	$(RM) -r build/RoboticsModel.app build/RoboticsModel.iconset build/RoboticsModel.icns
