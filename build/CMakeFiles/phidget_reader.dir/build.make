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
include CMakeFiles/phidget_reader.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/phidget_reader.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/phidget_reader.dir/flags.make

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o: CMakeFiles/phidget_reader.dir/flags.make
CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o: ../src/phidget_reader.cpp
CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o: ../manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/stijn/ROS/Grijper/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o -c /home/stijn/ROS/Grijper/src/phidget_reader.cpp

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/stijn/ROS/Grijper/src/phidget_reader.cpp > CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.i

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/stijn/ROS/Grijper/src/phidget_reader.cpp -o CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.s

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.requires:
.PHONY : CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.requires

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.provides: CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.requires
	$(MAKE) -f CMakeFiles/phidget_reader.dir/build.make CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.provides.build
.PHONY : CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.provides

CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.provides.build: CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o

# Object files for target phidget_reader
phidget_reader_OBJECTS = \
"CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o"

# External object files for target phidget_reader
phidget_reader_EXTERNAL_OBJECTS =

../bin/phidget_reader: CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o
../bin/phidget_reader: CMakeFiles/phidget_reader.dir/build.make
../bin/phidget_reader: CMakeFiles/phidget_reader.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/phidget_reader"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/phidget_reader.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/phidget_reader.dir/build: ../bin/phidget_reader
.PHONY : CMakeFiles/phidget_reader.dir/build

CMakeFiles/phidget_reader.dir/requires: CMakeFiles/phidget_reader.dir/src/phidget_reader.cpp.o.requires
.PHONY : CMakeFiles/phidget_reader.dir/requires

CMakeFiles/phidget_reader.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/phidget_reader.dir/cmake_clean.cmake
.PHONY : CMakeFiles/phidget_reader.dir/clean

CMakeFiles/phidget_reader.dir/depend:
	cd /home/stijn/ROS/Grijper/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stijn/ROS/Grijper /home/stijn/ROS/Grijper /home/stijn/ROS/Grijper/build /home/stijn/ROS/Grijper/build /home/stijn/ROS/Grijper/build/CMakeFiles/phidget_reader.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/phidget_reader.dir/depend

