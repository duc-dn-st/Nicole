# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/musashi/catkin_ws/MPCC/src/External/blasfeo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/example_s_riccati_recursion.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example_s_riccati_recursion.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make

examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o: examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make
examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o: ../examples/example_s_riccati_recursion.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o   -c /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_s_riccati_recursion.c

examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_s_riccati_recursion.c > CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.i

examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_s_riccati_recursion.c -o CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.s

examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o: examples/CMakeFiles/example_s_riccati_recursion.dir/flags.make
examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o: ../examples/tools.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/example_s_riccati_recursion.dir/tools.c.o   -c /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/tools.c

examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/example_s_riccati_recursion.dir/tools.c.i"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/tools.c > CMakeFiles/example_s_riccati_recursion.dir/tools.c.i

examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/example_s_riccati_recursion.dir/tools.c.s"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/tools.c -o CMakeFiles/example_s_riccati_recursion.dir/tools.c.s

# Object files for target example_s_riccati_recursion
example_s_riccati_recursion_OBJECTS = \
"CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o" \
"CMakeFiles/example_s_riccati_recursion.dir/tools.c.o"

# External object files for target example_s_riccati_recursion
example_s_riccati_recursion_EXTERNAL_OBJECTS =

examples/example_s_riccati_recursion: examples/CMakeFiles/example_s_riccati_recursion.dir/example_s_riccati_recursion.c.o
examples/example_s_riccati_recursion: examples/CMakeFiles/example_s_riccati_recursion.dir/tools.c.o
examples/example_s_riccati_recursion: examples/CMakeFiles/example_s_riccati_recursion.dir/build.make
examples/example_s_riccati_recursion: libblasfeo.a
examples/example_s_riccati_recursion: examples/CMakeFiles/example_s_riccati_recursion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable example_s_riccati_recursion"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_s_riccati_recursion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example_s_riccati_recursion.dir/build: examples/example_s_riccati_recursion

.PHONY : examples/CMakeFiles/example_s_riccati_recursion.dir/build

examples/CMakeFiles/example_s_riccati_recursion.dir/clean:
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example_s_riccati_recursion.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example_s_riccati_recursion.dir/clean

examples/CMakeFiles/example_s_riccati_recursion.dir/depend:
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/musashi/catkin_ws/MPCC/src/External/blasfeo /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples/CMakeFiles/example_s_riccati_recursion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example_s_riccati_recursion.dir/depend
