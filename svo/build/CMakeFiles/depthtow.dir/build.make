# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/worxli/rpg/rpg_svo/svo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/worxli/rpg/rpg_svo/svo/build

# Include any dependencies generated for this target.
include CMakeFiles/depthtow.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/depthtow.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/depthtow.dir/flags.make

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o: CMakeFiles/depthtow.dir/flags.make
CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o: ../test/getcloudfromdepth.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o -c /home/worxli/rpg/rpg_svo/svo/test/getcloudfromdepth.cpp

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/test/getcloudfromdepth.cpp > CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.i

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/test/getcloudfromdepth.cpp -o CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.s

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.requires:
.PHONY : CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.requires

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.provides: CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.requires
	$(MAKE) -f CMakeFiles/depthtow.dir/build.make CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.provides.build
.PHONY : CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.provides

CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.provides.build: CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o

# Object files for target depthtow
depthtow_OBJECTS = \
"CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o"

# External object files for target depthtow
depthtow_EXTERNAL_OBJECTS =

../bin/depthtow: CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o
../bin/depthtow: CMakeFiles/depthtow.dir/build.make
../bin/depthtow: /usr/local/lib/libopencv_core.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_imgproc.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_highgui.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_calib3d.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_viz.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_videostab.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_video.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_ts.a
../bin/depthtow: /usr/local/lib/libopencv_superres.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_stitching.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_photo.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_ocl.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_objdetect.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_nonfree.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_ml.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_legacy.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_imgproc.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_highgui.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_gpu.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_flann.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_features2d.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_core.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_contrib.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_calib3d.so.2.4.9
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libSM.so
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libICE.so
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libX11.so
../bin/depthtow: /usr/lib/x86_64-linux-gnu/libXext.so
../bin/depthtow: /usr/local/lib/libopencv_nonfree.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_ocl.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_gpu.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_photo.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_objdetect.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_legacy.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_calib3d.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_video.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_ml.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_features2d.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_highgui.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_imgproc.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_flann.so.2.4.9
../bin/depthtow: /usr/local/lib/libopencv_core.so.2.4.9
../bin/depthtow: CMakeFiles/depthtow.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/depthtow"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depthtow.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/depthtow.dir/build: ../bin/depthtow
.PHONY : CMakeFiles/depthtow.dir/build

CMakeFiles/depthtow.dir/requires: CMakeFiles/depthtow.dir/test/getcloudfromdepth.cpp.o.requires
.PHONY : CMakeFiles/depthtow.dir/requires

CMakeFiles/depthtow.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/depthtow.dir/cmake_clean.cmake
.PHONY : CMakeFiles/depthtow.dir/clean

CMakeFiles/depthtow.dir/depend:
	cd /home/worxli/rpg/rpg_svo/svo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo/build /home/worxli/rpg/rpg_svo/svo/build /home/worxli/rpg/rpg_svo/svo/build/CMakeFiles/depthtow.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/depthtow.dir/depend

