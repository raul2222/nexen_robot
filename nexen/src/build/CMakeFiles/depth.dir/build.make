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
CMAKE_SOURCE_DIR = /home/raul/Documents/libsynexen/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/raul/Documents/libsynexen/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/depth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/depth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/depth.dir/flags.make

CMakeFiles/depth.dir/depth.cpp.o: CMakeFiles/depth.dir/flags.make
CMakeFiles/depth.dir/depth.cpp.o: ../depth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/raul/Documents/libsynexen/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/depth.dir/depth.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth.dir/depth.cpp.o -c /home/raul/Documents/libsynexen/examples/depth.cpp

CMakeFiles/depth.dir/depth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth.dir/depth.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/raul/Documents/libsynexen/examples/depth.cpp > CMakeFiles/depth.dir/depth.cpp.i

CMakeFiles/depth.dir/depth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth.dir/depth.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/raul/Documents/libsynexen/examples/depth.cpp -o CMakeFiles/depth.dir/depth.cpp.s

# Object files for target depth
depth_OBJECTS = \
"CMakeFiles/depth.dir/depth.cpp.o"

# External object files for target depth
depth_EXTERNAL_OBJECTS =

depth: CMakeFiles/depth.dir/depth.cpp.o
depth: CMakeFiles/depth.dir/build.make
depth: /usr/local/lib/libopencv_dnn.so.4.4.0
depth: /usr/local/lib/libopencv_gapi.so.4.4.0
depth: /usr/local/lib/libopencv_highgui.so.4.4.0
depth: /usr/local/lib/libopencv_ml.so.4.4.0
depth: /usr/local/lib/libopencv_objdetect.so.4.4.0
depth: /usr/local/lib/libopencv_photo.so.4.4.0
depth: /usr/local/lib/libopencv_stitching.so.4.4.0
depth: /usr/local/lib/libopencv_video.so.4.4.0
depth: /usr/local/lib/libopencv_videoio.so.4.4.0
depth: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
depth: /usr/local/lib/libopencv_calib3d.so.4.4.0
depth: /usr/local/lib/libopencv_features2d.so.4.4.0
depth: /usr/local/lib/libopencv_flann.so.4.4.0
depth: /usr/local/lib/libopencv_imgproc.so.4.4.0
depth: /usr/local/lib/libopencv_core.so.4.4.0
depth: CMakeFiles/depth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/raul/Documents/libsynexen/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable depth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/depth.dir/build: depth

.PHONY : CMakeFiles/depth.dir/build

CMakeFiles/depth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/depth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/depth.dir/clean

CMakeFiles/depth.dir/depend:
	cd /home/raul/Documents/libsynexen/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/raul/Documents/libsynexen/examples /home/raul/Documents/libsynexen/examples /home/raul/Documents/libsynexen/examples/build /home/raul/Documents/libsynexen/examples/build /home/raul/Documents/libsynexen/examples/build/CMakeFiles/depth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/depth.dir/depend

