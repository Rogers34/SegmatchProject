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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rogers/yj_ws/SegmatchProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rogers/yj_ws/SegmatchProject/build

# Include any dependencies generated for this target.
include CMakeFiles/segmatch.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segmatch.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segmatch.dir/flags.make

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o: CMakeFiles/segmatch.dir/flags.make
CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o: ../src/segmatch/SegMatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rogers/yj_ws/SegmatchProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o -c /home/rogers/yj_ws/SegmatchProject/src/segmatch/SegMatch.cpp

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rogers/yj_ws/SegmatchProject/src/segmatch/SegMatch.cpp > CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.i

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rogers/yj_ws/SegmatchProject/src/segmatch/SegMatch.cpp -o CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.s

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.requires:

.PHONY : CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.requires

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.provides: CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmatch.dir/build.make CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.provides.build
.PHONY : CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.provides

CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.provides.build: CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o


CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o: CMakeFiles/segmatch.dir/flags.make
CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o: ../src/segmatch/segmenters/region_growing_segmenter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rogers/yj_ws/SegmatchProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o -c /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/region_growing_segmenter.cpp

CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/region_growing_segmenter.cpp > CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.i

CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/region_growing_segmenter.cpp -o CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.s

CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.requires:

.PHONY : CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.requires

CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.provides: CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmatch.dir/build.make CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.provides.build
.PHONY : CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.provides

CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.provides.build: CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o


CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o: CMakeFiles/segmatch.dir/flags.make
CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o: ../src/segmatch/segmenters/euclidean_segmenter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rogers/yj_ws/SegmatchProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o -c /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/euclidean_segmenter.cpp

CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/euclidean_segmenter.cpp > CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.i

CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rogers/yj_ws/SegmatchProject/src/segmatch/segmenters/euclidean_segmenter.cpp -o CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.s

CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.requires:

.PHONY : CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.requires

CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.provides: CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.requires
	$(MAKE) -f CMakeFiles/segmatch.dir/build.make CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.provides.build
.PHONY : CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.provides

CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.provides.build: CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o


# Object files for target segmatch
segmatch_OBJECTS = \
"CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o" \
"CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o" \
"CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o"

# External object files for target segmatch
segmatch_EXTERNAL_OBJECTS =

libsegmatch.so: CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o
libsegmatch.so: CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o
libsegmatch.so: CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o
libsegmatch.so: CMakeFiles/segmatch.dir/build.make
libsegmatch.so: /usr/local/lib/libopencv_videostab.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_ts.a
libsegmatch.so: /usr/local/lib/libopencv_superres.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_stitching.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_contrib.so.2.4.9
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libGL.so
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libSM.so
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libICE.so
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libX11.so
libsegmatch.so: /usr/lib/x86_64-linux-gnu/libXext.so
libsegmatch.so: /usr/local/lib/libopencv_nonfree.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_ocl.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_gpu.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_photo.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_objdetect.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_legacy.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_video.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_ml.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_calib3d.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_features2d.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_highgui.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_imgproc.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_flann.so.2.4.9
libsegmatch.so: /usr/local/lib/libopencv_core.so.2.4.9
libsegmatch.so: CMakeFiles/segmatch.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rogers/yj_ws/SegmatchProject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libsegmatch.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segmatch.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segmatch.dir/build: libsegmatch.so

.PHONY : CMakeFiles/segmatch.dir/build

CMakeFiles/segmatch.dir/requires: CMakeFiles/segmatch.dir/src/segmatch/SegMatch.cpp.o.requires
CMakeFiles/segmatch.dir/requires: CMakeFiles/segmatch.dir/src/segmatch/segmenters/region_growing_segmenter.cpp.o.requires
CMakeFiles/segmatch.dir/requires: CMakeFiles/segmatch.dir/src/segmatch/segmenters/euclidean_segmenter.cpp.o.requires

.PHONY : CMakeFiles/segmatch.dir/requires

CMakeFiles/segmatch.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segmatch.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segmatch.dir/clean

CMakeFiles/segmatch.dir/depend:
	cd /home/rogers/yj_ws/SegmatchProject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rogers/yj_ws/SegmatchProject /home/rogers/yj_ws/SegmatchProject /home/rogers/yj_ws/SegmatchProject/build /home/rogers/yj_ws/SegmatchProject/build /home/rogers/yj_ws/SegmatchProject/build/CMakeFiles/segmatch.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segmatch.dir/depend
