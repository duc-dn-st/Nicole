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
CMAKE_SOURCE_DIR = /home/musashi/catkin_ws/MPCC/src/External/hpipm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/musashi/catkin_ws/MPCC/src/External/hpipm/build

# Include any dependencies generated for this target.
include test_problems/CMakeFiles/d_tree_ocp_qp.dir/depend.make

# Include the progress variables for this target.
include test_problems/CMakeFiles/d_tree_ocp_qp.dir/progress.make

# Include the compile flags for this target's objects.
include test_problems/CMakeFiles/d_tree_ocp_qp.dir/flags.make

test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o: test_problems/CMakeFiles/d_tree_ocp_qp.dir/flags.make
test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o: ../test_problems/test_d_tree_ocp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/hpipm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o   -c /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/test_d_tree_ocp.c

test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.i"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/test_d_tree_ocp.c > CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.i

test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.s"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/test_d_tree_ocp.c -o CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.s

test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o: test_problems/CMakeFiles/d_tree_ocp_qp.dir/flags.make
test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o: ../test_problems/d_tools.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/hpipm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o   -c /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/d_tools.c

test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.i"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/d_tools.c > CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.i

test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.s"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems/d_tools.c -o CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.s

# Object files for target d_tree_ocp_qp
d_tree_ocp_qp_OBJECTS = \
"CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o" \
"CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o"

# External object files for target d_tree_ocp_qp
d_tree_ocp_qp_EXTERNAL_OBJECTS =

test_problems/d_tree_ocp_qp: test_problems/CMakeFiles/d_tree_ocp_qp.dir/test_d_tree_ocp.c.o
test_problems/d_tree_ocp_qp: test_problems/CMakeFiles/d_tree_ocp_qp.dir/d_tools.c.o
test_problems/d_tree_ocp_qp: test_problems/CMakeFiles/d_tree_ocp_qp.dir/build.make
test_problems/d_tree_ocp_qp: libhpipm.a
test_problems/d_tree_ocp_qp: /home/musashi/catkin_ws/MPCC/src/External/blasfeo/lib/lib/libblasfeo.a
test_problems/d_tree_ocp_qp: test_problems/CMakeFiles/d_tree_ocp_qp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/musashi/catkin_ws/MPCC/src/External/hpipm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable d_tree_ocp_qp"
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/d_tree_ocp_qp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test_problems/CMakeFiles/d_tree_ocp_qp.dir/build: test_problems/d_tree_ocp_qp

.PHONY : test_problems/CMakeFiles/d_tree_ocp_qp.dir/build

test_problems/CMakeFiles/d_tree_ocp_qp.dir/clean:
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems && $(CMAKE_COMMAND) -P CMakeFiles/d_tree_ocp_qp.dir/cmake_clean.cmake
.PHONY : test_problems/CMakeFiles/d_tree_ocp_qp.dir/clean

test_problems/CMakeFiles/d_tree_ocp_qp.dir/depend:
	cd /home/musashi/catkin_ws/MPCC/src/External/hpipm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/musashi/catkin_ws/MPCC/src/External/hpipm /home/musashi/catkin_ws/MPCC/src/External/hpipm/test_problems /home/musashi/catkin_ws/MPCC/src/External/hpipm/build /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems /home/musashi/catkin_ws/MPCC/src/External/hpipm/build/test_problems/CMakeFiles/d_tree_ocp_qp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test_problems/CMakeFiles/d_tree_ocp_qp.dir/depend

