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
CMAKE_SOURCE_DIR = /home/xxz/catkin_ws/src/auto_flight

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xxz/catkin_ws/src/auto_flight/build

# Include any dependencies generated for this target.
include test/CMakeFiles/setting_destination.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/setting_destination.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/setting_destination.dir/flags.make

test/CMakeFiles/setting_destination.dir/set_destination.cpp.o: test/CMakeFiles/setting_destination.dir/flags.make
test/CMakeFiles/setting_destination.dir/set_destination.cpp.o: ../test/set_destination.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xxz/catkin_ws/src/auto_flight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/setting_destination.dir/set_destination.cpp.o"
	cd /home/xxz/catkin_ws/src/auto_flight/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/setting_destination.dir/set_destination.cpp.o -c /home/xxz/catkin_ws/src/auto_flight/test/set_destination.cpp

test/CMakeFiles/setting_destination.dir/set_destination.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/setting_destination.dir/set_destination.cpp.i"
	cd /home/xxz/catkin_ws/src/auto_flight/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xxz/catkin_ws/src/auto_flight/test/set_destination.cpp > CMakeFiles/setting_destination.dir/set_destination.cpp.i

test/CMakeFiles/setting_destination.dir/set_destination.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/setting_destination.dir/set_destination.cpp.s"
	cd /home/xxz/catkin_ws/src/auto_flight/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xxz/catkin_ws/src/auto_flight/test/set_destination.cpp -o CMakeFiles/setting_destination.dir/set_destination.cpp.s

test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.requires:

.PHONY : test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.requires

test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.provides: test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/setting_destination.dir/build.make test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.provides.build
.PHONY : test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.provides

test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.provides.build: test/CMakeFiles/setting_destination.dir/set_destination.cpp.o


# Object files for target setting_destination
setting_destination_OBJECTS = \
"CMakeFiles/setting_destination.dir/set_destination.cpp.o"

# External object files for target setting_destination
setting_destination_EXTERNAL_OBJECTS =

devel/lib/auto_flight/setting_destination: test/CMakeFiles/setting_destination.dir/set_destination.cpp.o
devel/lib/auto_flight/setting_destination: test/CMakeFiles/setting_destination.dir/build.make
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libopencv_core3.so.3.3.1
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.3.1
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.3.1
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libmavros.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libGeographic.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libmavconn.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libeigen_conversions.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/auto_flight/setting_destination: /usr/lib/libPocoFoundation.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libroslib.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/librospack.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libtf.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libtf2_ros.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libactionlib.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libtf2.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libcamera_info_manager.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libcamera_calibration_parsers.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/librostime.so
devel/lib/auto_flight/setting_destination: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/auto_flight/setting_destination: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/auto_flight/setting_destination: test/CMakeFiles/setting_destination.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xxz/catkin_ws/src/auto_flight/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../devel/lib/auto_flight/setting_destination"
	cd /home/xxz/catkin_ws/src/auto_flight/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/setting_destination.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/setting_destination.dir/build: devel/lib/auto_flight/setting_destination

.PHONY : test/CMakeFiles/setting_destination.dir/build

test/CMakeFiles/setting_destination.dir/requires: test/CMakeFiles/setting_destination.dir/set_destination.cpp.o.requires

.PHONY : test/CMakeFiles/setting_destination.dir/requires

test/CMakeFiles/setting_destination.dir/clean:
	cd /home/xxz/catkin_ws/src/auto_flight/build/test && $(CMAKE_COMMAND) -P CMakeFiles/setting_destination.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/setting_destination.dir/clean

test/CMakeFiles/setting_destination.dir/depend:
	cd /home/xxz/catkin_ws/src/auto_flight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xxz/catkin_ws/src/auto_flight /home/xxz/catkin_ws/src/auto_flight/test /home/xxz/catkin_ws/src/auto_flight/build /home/xxz/catkin_ws/src/auto_flight/build/test /home/xxz/catkin_ws/src/auto_flight/build/test/CMakeFiles/setting_destination.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/setting_destination.dir/depend

