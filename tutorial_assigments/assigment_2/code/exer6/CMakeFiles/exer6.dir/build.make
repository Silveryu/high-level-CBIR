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
CMAKE_SOURCE_DIR = /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6

# Include any dependencies generated for this target.
include CMakeFiles/exer6.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/exer6.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exer6.dir/flags.make

CMakeFiles/exer6.dir/hist.cpp.o: CMakeFiles/exer6.dir/flags.make
CMakeFiles/exer6.dir/hist.cpp.o: hist.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exer6.dir/hist.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exer6.dir/hist.cpp.o -c /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/hist.cpp

CMakeFiles/exer6.dir/hist.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exer6.dir/hist.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/hist.cpp > CMakeFiles/exer6.dir/hist.cpp.i

CMakeFiles/exer6.dir/hist.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exer6.dir/hist.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/hist.cpp -o CMakeFiles/exer6.dir/hist.cpp.s

CMakeFiles/exer6.dir/hist.cpp.o.requires:

.PHONY : CMakeFiles/exer6.dir/hist.cpp.o.requires

CMakeFiles/exer6.dir/hist.cpp.o.provides: CMakeFiles/exer6.dir/hist.cpp.o.requires
	$(MAKE) -f CMakeFiles/exer6.dir/build.make CMakeFiles/exer6.dir/hist.cpp.o.provides.build
.PHONY : CMakeFiles/exer6.dir/hist.cpp.o.provides

CMakeFiles/exer6.dir/hist.cpp.o.provides.build: CMakeFiles/exer6.dir/hist.cpp.o


# Object files for target exer6
exer6_OBJECTS = \
"CMakeFiles/exer6.dir/hist.cpp.o"

# External object files for target exer6
exer6_EXTERNAL_OBJECTS =

exer6: CMakeFiles/exer6.dir/hist.cpp.o
exer6: CMakeFiles/exer6.dir/build.make
exer6: /usr/local/lib/libopencv_dnn.so.4.0.0
exer6: /usr/local/lib/libopencv_ml.so.4.0.0
exer6: /usr/local/lib/libopencv_objdetect.so.4.0.0
exer6: /usr/local/lib/libopencv_shape.so.4.0.0
exer6: /usr/local/lib/libopencv_stitching.so.4.0.0
exer6: /usr/local/lib/libopencv_superres.so.4.0.0
exer6: /usr/local/lib/libopencv_videostab.so.4.0.0
exer6: /usr/local/lib/libopencv_photo.so.4.0.0
exer6: /usr/local/lib/libopencv_video.so.4.0.0
exer6: /usr/local/lib/libopencv_calib3d.so.4.0.0
exer6: /usr/local/lib/libopencv_features2d.so.4.0.0
exer6: /usr/local/lib/libopencv_flann.so.4.0.0
exer6: /usr/local/lib/libopencv_highgui.so.4.0.0
exer6: /usr/local/lib/libopencv_videoio.so.4.0.0
exer6: /usr/local/lib/libopencv_imgcodecs.so.4.0.0
exer6: /usr/local/lib/libopencv_imgproc.so.4.0.0
exer6: /usr/local/lib/libopencv_core.so.4.0.0
exer6: CMakeFiles/exer6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exer6"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exer6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exer6.dir/build: exer6

.PHONY : CMakeFiles/exer6.dir/build

CMakeFiles/exer6.dir/requires: CMakeFiles/exer6.dir/hist.cpp.o.requires

.PHONY : CMakeFiles/exer6.dir/requires

CMakeFiles/exer6.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exer6.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exer6.dir/clean

CMakeFiles/exer6.dir/depend:
	cd /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6 /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6 /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6 /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6 /home/rute/Documents/cadeiras/5ano/vc/vc1819-76505-76532/tutorial_assigments/assigment_2/code/exer6/CMakeFiles/exer6.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exer6.dir/depend

