# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/stijn/ROS/Grijper

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stijn/ROS/Grijper/build

# Include any dependencies generated for this target.
include CMakeFiles/gripper_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gripper_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gripper_controller.dir/flags.make

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o: CMakeFiles/gripper_controller.dir/flags.make
CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o: ../src/gripper_controller.cpp
CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o: ../manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/stijn/ROS/Grijper/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o -c /home/stijn/ROS/Grijper/src/gripper_controller.cpp

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/stijn/ROS/Grijper/src/gripper_controller.cpp > CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.i

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/stijn/ROS/Grijper/src/gripper_controller.cpp -o CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.s

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.requires:
.PHONY : CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.requires

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.provides: CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.requires
	$(MAKE) -f CMakeFiles/gripper_controller.dir/build.make CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.provides.build
.PHONY : CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.provides

CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.provides.build: CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o

# Object files for target gripper_controller
gripper_controller_OBJECTS = \
"CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o"

# External object files for target gripper_controller
gripper_controller_EXTERNAL_OBJECTS =

../bin/gripper_controller: CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o
../bin/gripper_controller: CMakeFiles/gripper_controller.dir/build.make
../bin/gripper_controller: CMakeFiles/gripper_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/gripper_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gripper_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gripper_controller.dir/build: ../bin/gripper_controller
.PHONY : CMakeFiles/gripper_controller.dir/build

CMakeFiles/gripper_controller.dir/requires: CMakeFiles/gripper_controller.dir/src/gripper_controller.cpp.o.requires
.PHONY : CMakeFiles/gripper_controller.dir/requires

CMakeFiles/gripper_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gripper_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gripper_controller.dir/clean

CMakeFiles/gripper_controller.dir/depend:
	cd /home/stijn/ROS/Grijper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stijn/ROS/Grijper /home/stijn/ROS/Grijper /home/stijn/ROS/Grijper/build /home/stijn/ROS/Grijper/build /home/stijn/ROS/Grijper/build/CMakeFiles/gripper_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gripper_controller.dir/depend

