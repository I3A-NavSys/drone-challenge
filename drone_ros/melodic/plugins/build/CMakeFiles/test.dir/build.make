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
CMAKE_SOURCE_DIR = /opt/ros/melodic/share/drone_ros/plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/ros/melodic/share/drone_ros/plugins/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/test.cc.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/test.cc.o: ../test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/ros/melodic/share/drone_ros/plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/test.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/test.cc.o -c /opt/ros/melodic/share/drone_ros/plugins/test.cc

CMakeFiles/test.dir/test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/test.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/ros/melodic/share/drone_ros/plugins/test.cc > CMakeFiles/test.dir/test.cc.i

CMakeFiles/test.dir/test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/test.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/ros/melodic/share/drone_ros/plugins/test.cc -o CMakeFiles/test.dir/test.cc.s

CMakeFiles/test.dir/test.cc.o.requires:

.PHONY : CMakeFiles/test.dir/test.cc.o.requires

CMakeFiles/test.dir/test.cc.o.provides: CMakeFiles/test.dir/test.cc.o.requires
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/test.cc.o.provides.build
.PHONY : CMakeFiles/test.dir/test.cc.o.provides

CMakeFiles/test.dir/test.cc.o.provides.build: CMakeFiles/test.dir/test.cc.o


# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/test.cc.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

libtest.so: CMakeFiles/test.dir/test.cc.o
libtest.so: CMakeFiles/test.dir/build.make
libtest.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libtest.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libtest.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libtest.so: /usr/lib/x86_64-linux-gnu/libblas.so
libtest.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libtest.so: /usr/lib/x86_64-linux-gnu/libblas.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtest.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libtest.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libtest.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
libtest.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
libtest.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.0.1
libtest.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.0.0
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtest.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libtest.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtest.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libtest.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtest.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtest.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtest.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libtest.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libtest.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libtest.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libtest.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
libtest.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libtest.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libtest.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libtest.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libtest.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libtest.so: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/ros/melodic/share/drone_ros/plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libtest.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: libtest.so

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/requires: CMakeFiles/test.dir/test.cc.o.requires

.PHONY : CMakeFiles/test.dir/requires

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /opt/ros/melodic/share/drone_ros/plugins/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/ros/melodic/share/drone_ros/plugins /opt/ros/melodic/share/drone_ros/plugins /opt/ros/melodic/share/drone_ros/plugins/build /opt/ros/melodic/share/drone_ros/plugins/build /opt/ros/melodic/share/drone_ros/plugins/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

