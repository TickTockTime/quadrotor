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
CMAKE_SOURCE_DIR = /home/tardis/tardis_workspace/quadrotor/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tardis/tardis_workspace/quadrotor/build

# Include any dependencies generated for this target.
include data_read/CMakeFiles/fly_test.dir/depend.make

# Include the progress variables for this target.
include data_read/CMakeFiles/fly_test.dir/progress.make

# Include the compile flags for this target's objects.
include data_read/CMakeFiles/fly_test.dir/flags.make

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o: data_read/CMakeFiles/fly_test.dir/flags.make
data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o: /home/tardis/tardis_workspace/quadrotor/src/data_read/src/fly_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tardis/tardis_workspace/quadrotor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o"
	cd /home/tardis/tardis_workspace/quadrotor/build/data_read && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fly_test.dir/src/fly_test.cpp.o -c /home/tardis/tardis_workspace/quadrotor/src/data_read/src/fly_test.cpp

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fly_test.dir/src/fly_test.cpp.i"
	cd /home/tardis/tardis_workspace/quadrotor/build/data_read && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tardis/tardis_workspace/quadrotor/src/data_read/src/fly_test.cpp > CMakeFiles/fly_test.dir/src/fly_test.cpp.i

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fly_test.dir/src/fly_test.cpp.s"
	cd /home/tardis/tardis_workspace/quadrotor/build/data_read && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tardis/tardis_workspace/quadrotor/src/data_read/src/fly_test.cpp -o CMakeFiles/fly_test.dir/src/fly_test.cpp.s

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.requires:

.PHONY : data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.requires

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.provides: data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.requires
	$(MAKE) -f data_read/CMakeFiles/fly_test.dir/build.make data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.provides.build
.PHONY : data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.provides

data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.provides.build: data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o


# Object files for target fly_test
fly_test_OBJECTS = \
"CMakeFiles/fly_test.dir/src/fly_test.cpp.o"

# External object files for target fly_test
fly_test_EXTERNAL_OBJECTS =

/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: data_read/CMakeFiles/fly_test.dir/build.make
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/libroscpp.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/librosconsole.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/librostime.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /opt/ros/kinetic/lib/libcpp_common.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test: data_read/CMakeFiles/fly_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tardis/tardis_workspace/quadrotor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test"
	cd /home/tardis/tardis_workspace/quadrotor/build/data_read && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fly_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
data_read/CMakeFiles/fly_test.dir/build: /home/tardis/tardis_workspace/quadrotor/devel/lib/data_read/fly_test

.PHONY : data_read/CMakeFiles/fly_test.dir/build

data_read/CMakeFiles/fly_test.dir/requires: data_read/CMakeFiles/fly_test.dir/src/fly_test.cpp.o.requires

.PHONY : data_read/CMakeFiles/fly_test.dir/requires

data_read/CMakeFiles/fly_test.dir/clean:
	cd /home/tardis/tardis_workspace/quadrotor/build/data_read && $(CMAKE_COMMAND) -P CMakeFiles/fly_test.dir/cmake_clean.cmake
.PHONY : data_read/CMakeFiles/fly_test.dir/clean

data_read/CMakeFiles/fly_test.dir/depend:
	cd /home/tardis/tardis_workspace/quadrotor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tardis/tardis_workspace/quadrotor/src /home/tardis/tardis_workspace/quadrotor/src/data_read /home/tardis/tardis_workspace/quadrotor/build /home/tardis/tardis_workspace/quadrotor/build/data_read /home/tardis/tardis_workspace/quadrotor/build/data_read/CMakeFiles/fly_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : data_read/CMakeFiles/fly_test.dir/depend

