# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build

# Include any dependencies generated for this target.
include tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/depend.make

# Include the progress variables for this target.
include tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/progress.make

# Include the compile flags for this target's objects.
include tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/flags.make

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/flags.make
tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o: /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/src/point_tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o"
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o -c /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/src/point_tracking.cpp

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_tracking.dir/src/point_tracking.cpp.i"
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/src/point_tracking.cpp > CMakeFiles/point_tracking.dir/src/point_tracking.cpp.i

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_tracking.dir/src/point_tracking.cpp.s"
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/src/point_tracking.cpp -o CMakeFiles/point_tracking.dir/src/point_tracking.cpp.s

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.requires:

.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.requires

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.provides: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.requires
	$(MAKE) -f tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/build.make tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.provides.build
.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.provides

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.provides.build: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o


# Object files for target point_tracking
point_tracking_OBJECTS = \
"CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o"

# External object files for target point_tracking
point_tracking_EXTERNAL_OBJECTS =

/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/build.make
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libimage_transport.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libmessage_filters.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libclass_loader.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/libPocoFoundation.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libdl.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libroscpp.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libroslib.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/librospack.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libcv_bridge.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/librosconsole.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libeigen_conversions.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/librostime.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /opt/ros/melodic/lib/libcpp_common.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking"
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/build: /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/devel/lib/tek4030_visual_servoing_point_tracking/point_tracking

.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/build

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/requires: tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/src/point_tracking.cpp.o.requires

.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/requires

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/clean:
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking && $(CMAKE_COMMAND) -P CMakeFiles/point_tracking.dir/cmake_clean.cmake
.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/clean

tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/depend:
	cd /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/src/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking /home/adrienk/TEK4030-A2/exercise_2/ros_intro_ws/build/tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tek4030_visual_servoing/tek4030_visual_servoing_point_tracking/CMakeFiles/point_tracking.dir/depend

