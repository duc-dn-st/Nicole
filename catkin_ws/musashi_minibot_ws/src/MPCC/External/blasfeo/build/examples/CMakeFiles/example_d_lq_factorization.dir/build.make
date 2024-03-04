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
include examples/CMakeFiles/example_d_lq_factorization.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example_d_lq_factorization.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example_d_lq_factorization.dir/flags.make

examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o: examples/CMakeFiles/example_d_lq_factorization.dir/flags.make
examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o: ../examples/example_d_lq_factorization.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o   -c /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_d_lq_factorization.c

examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.i"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_d_lq_factorization.c > CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.i

examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.s"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples/example_d_lq_factorization.c -o CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.s

# Object files for target example_d_lq_factorization
example_d_lq_factorization_OBJECTS = \
"CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o"

# External object files for target example_d_lq_factorization
example_d_lq_factorization_EXTERNAL_OBJECTS =

examples/example_d_lq_factorization: examples/CMakeFiles/example_d_lq_factorization.dir/example_d_lq_factorization.c.o
examples/example_d_lq_factorization: examples/CMakeFiles/example_d_lq_factorization.dir/build.make
examples/example_d_lq_factorization: libblasfeo.a
examples/example_d_lq_factorization: examples/CMakeFiles/example_d_lq_factorization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable example_d_lq_factorization"
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_d_lq_factorization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example_d_lq_factorization.dir/build: examples/example_d_lq_factorization

.PHONY : examples/CMakeFiles/example_d_lq_factorization.dir/build

examples/CMakeFiles/example_d_lq_factorization.dir/clean:
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/example_d_lq_factorization.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example_d_lq_factorization.dir/clean

examples/CMakeFiles/example_d_lq_factorization.dir/depend:
	cd /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/musashi/catkin_ws/MPCC/src/External/blasfeo /home/musashi/catkin_ws/MPCC/src/External/blasfeo/examples /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples /home/musashi/catkin_ws/MPCC/src/External/blasfeo/build/examples/CMakeFiles/example_d_lq_factorization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example_d_lq_factorization.dir/depend

