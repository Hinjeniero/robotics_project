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
CMAKE_SOURCE_DIR = /home/hinjeniero/robotics_project/supervisor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hinjeniero/robotics_project/supervisor

# Utility rule file for supervisor_automoc.

# Include the progress variables for this target.
include src/CMakeFiles/supervisor_automoc.dir/progress.make

src/CMakeFiles/supervisor_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/hinjeniero/robotics_project/supervisor/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target supervisor"
	cd /home/hinjeniero/robotics_project/supervisor/src && /usr/bin/cmake -E cmake_autogen /home/hinjeniero/robotics_project/supervisor/src/CMakeFiles/supervisor_automoc.dir/ ""

supervisor_automoc: src/CMakeFiles/supervisor_automoc
supervisor_automoc: src/CMakeFiles/supervisor_automoc.dir/build.make

.PHONY : supervisor_automoc

# Rule to build all files generated by this target.
src/CMakeFiles/supervisor_automoc.dir/build: supervisor_automoc

.PHONY : src/CMakeFiles/supervisor_automoc.dir/build

src/CMakeFiles/supervisor_automoc.dir/clean:
	cd /home/hinjeniero/robotics_project/supervisor/src && $(CMAKE_COMMAND) -P CMakeFiles/supervisor_automoc.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/supervisor_automoc.dir/clean

src/CMakeFiles/supervisor_automoc.dir/depend:
	cd /home/hinjeniero/robotics_project/supervisor && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hinjeniero/robotics_project/supervisor /home/hinjeniero/robotics_project/supervisor/src /home/hinjeniero/robotics_project/supervisor /home/hinjeniero/robotics_project/supervisor/src /home/hinjeniero/robotics_project/supervisor/src/CMakeFiles/supervisor_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/supervisor_automoc.dir/depend

