# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/isler/catkin_ws/src/librealsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/isler/catkin_ws/src/librealsense

# Include any dependencies generated for this target.
include unit-tests/CMakeFiles/SR300-live-test.dir/depend.make

# Include the progress variables for this target.
include unit-tests/CMakeFiles/SR300-live-test.dir/progress.make

# Include the compile flags for this target's objects.
include unit-tests/CMakeFiles/SR300-live-test.dir/flags.make

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o: unit-tests/CMakeFiles/SR300-live-test.dir/flags.make
unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o: unit-tests/unit-tests-live.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/isler/catkin_ws/src/librealsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o -c /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live.cpp

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.i"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live.cpp > CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.i

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.s"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live.cpp -o CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.s

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.requires:

.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.requires

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.provides: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.requires
	$(MAKE) -f unit-tests/CMakeFiles/SR300-live-test.dir/build.make unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.provides.build
.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.provides

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.provides.build: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o


unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o: unit-tests/CMakeFiles/SR300-live-test.dir/flags.make
unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o: unit-tests/unit-tests-live-sr300.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/isler/catkin_ws/src/librealsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o -c /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live-sr300.cpp

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.i"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live-sr300.cpp > CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.i

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.s"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/isler/catkin_ws/src/librealsense/unit-tests/unit-tests-live-sr300.cpp -o CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.s

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.requires:

.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.requires

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.provides: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.requires
	$(MAKE) -f unit-tests/CMakeFiles/SR300-live-test.dir/build.make unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.provides.build
.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.provides

unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.provides.build: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o


# Object files for target SR300-live-test
SR300__live__test_OBJECTS = \
"CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o" \
"CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o"

# External object files for target SR300-live-test
SR300__live__test_EXTERNAL_OBJECTS =

devel/lib/librealsense/SR300-live-test: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o
devel/lib/librealsense/SR300-live-test: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o
devel/lib/librealsense/SR300-live-test: unit-tests/CMakeFiles/SR300-live-test.dir/build.make
devel/lib/librealsense/SR300-live-test: devel/lib/librealsense.so.1.12.1
devel/lib/librealsense/SR300-live-test: unit-tests/CMakeFiles/SR300-live-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/isler/catkin_ws/src/librealsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../devel/lib/librealsense/SR300-live-test"
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SR300-live-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unit-tests/CMakeFiles/SR300-live-test.dir/build: devel/lib/librealsense/SR300-live-test

.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/build

unit-tests/CMakeFiles/SR300-live-test.dir/requires: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live.cpp.o.requires
unit-tests/CMakeFiles/SR300-live-test.dir/requires: unit-tests/CMakeFiles/SR300-live-test.dir/unit-tests-live-sr300.cpp.o.requires

.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/requires

unit-tests/CMakeFiles/SR300-live-test.dir/clean:
	cd /home/isler/catkin_ws/src/librealsense/unit-tests && $(CMAKE_COMMAND) -P CMakeFiles/SR300-live-test.dir/cmake_clean.cmake
.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/clean

unit-tests/CMakeFiles/SR300-live-test.dir/depend:
	cd /home/isler/catkin_ws/src/librealsense && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/isler/catkin_ws/src/librealsense /home/isler/catkin_ws/src/librealsense/unit-tests /home/isler/catkin_ws/src/librealsense /home/isler/catkin_ws/src/librealsense/unit-tests /home/isler/catkin_ws/src/librealsense/unit-tests/CMakeFiles/SR300-live-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unit-tests/CMakeFiles/SR300-live-test.dir/depend

