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
CMAKE_BINARY_DIR = /home/worxli/rpg/rpg_svo/svo

# Include any dependencies generated for this target.
include CMakeFiles/svo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/svo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/svo.dir/flags.make

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o: src/frame_handler_mono.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/frame_handler_mono.cpp

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/frame_handler_mono.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/frame_handler_mono.cpp > CMakeFiles/svo.dir/src/frame_handler_mono.cpp.i

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/frame_handler_mono.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/frame_handler_mono.cpp -o CMakeFiles/svo.dir/src/frame_handler_mono.cpp.s

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.requires

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.provides: CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.provides

CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.provides.build: CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o

CMakeFiles/svo.dir/src/frame_handler_base.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/frame_handler_base.cpp.o: src/frame_handler_base.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/frame_handler_base.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/frame_handler_base.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/frame_handler_base.cpp

CMakeFiles/svo.dir/src/frame_handler_base.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/frame_handler_base.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/frame_handler_base.cpp > CMakeFiles/svo.dir/src/frame_handler_base.cpp.i

CMakeFiles/svo.dir/src/frame_handler_base.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/frame_handler_base.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/frame_handler_base.cpp -o CMakeFiles/svo.dir/src/frame_handler_base.cpp.s

CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.requires

CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.provides: CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.provides

CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.provides.build: CMakeFiles/svo.dir/src/frame_handler_base.cpp.o

CMakeFiles/svo.dir/src/frame.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/frame.cpp.o: src/frame.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/frame.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/frame.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/frame.cpp

CMakeFiles/svo.dir/src/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/frame.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/frame.cpp > CMakeFiles/svo.dir/src/frame.cpp.i

CMakeFiles/svo.dir/src/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/frame.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/frame.cpp -o CMakeFiles/svo.dir/src/frame.cpp.s

CMakeFiles/svo.dir/src/frame.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/frame.cpp.o.requires

CMakeFiles/svo.dir/src/frame.cpp.o.provides: CMakeFiles/svo.dir/src/frame.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/frame.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/frame.cpp.o.provides

CMakeFiles/svo.dir/src/frame.cpp.o.provides.build: CMakeFiles/svo.dir/src/frame.cpp.o

CMakeFiles/svo.dir/src/point.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/point.cpp.o: src/point.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/point.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/point.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/point.cpp

CMakeFiles/svo.dir/src/point.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/point.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/point.cpp > CMakeFiles/svo.dir/src/point.cpp.i

CMakeFiles/svo.dir/src/point.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/point.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/point.cpp -o CMakeFiles/svo.dir/src/point.cpp.s

CMakeFiles/svo.dir/src/point.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/point.cpp.o.requires

CMakeFiles/svo.dir/src/point.cpp.o.provides: CMakeFiles/svo.dir/src/point.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/point.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/point.cpp.o.provides

CMakeFiles/svo.dir/src/point.cpp.o.provides.build: CMakeFiles/svo.dir/src/point.cpp.o

CMakeFiles/svo.dir/src/map.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/map.cpp.o: src/map.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/map.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/map.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/map.cpp

CMakeFiles/svo.dir/src/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/map.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/map.cpp > CMakeFiles/svo.dir/src/map.cpp.i

CMakeFiles/svo.dir/src/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/map.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/map.cpp -o CMakeFiles/svo.dir/src/map.cpp.s

CMakeFiles/svo.dir/src/map.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/map.cpp.o.requires

CMakeFiles/svo.dir/src/map.cpp.o.provides: CMakeFiles/svo.dir/src/map.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/map.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/map.cpp.o.provides

CMakeFiles/svo.dir/src/map.cpp.o.provides.build: CMakeFiles/svo.dir/src/map.cpp.o

CMakeFiles/svo.dir/src/pose_optimizer.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/pose_optimizer.cpp.o: src/pose_optimizer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/pose_optimizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/pose_optimizer.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/pose_optimizer.cpp

CMakeFiles/svo.dir/src/pose_optimizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/pose_optimizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/pose_optimizer.cpp > CMakeFiles/svo.dir/src/pose_optimizer.cpp.i

CMakeFiles/svo.dir/src/pose_optimizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/pose_optimizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/pose_optimizer.cpp -o CMakeFiles/svo.dir/src/pose_optimizer.cpp.s

CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.requires

CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.provides: CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.provides

CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.provides.build: CMakeFiles/svo.dir/src/pose_optimizer.cpp.o

CMakeFiles/svo.dir/src/initialization.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/initialization.cpp.o: src/initialization.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/initialization.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/initialization.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/initialization.cpp

CMakeFiles/svo.dir/src/initialization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/initialization.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/initialization.cpp > CMakeFiles/svo.dir/src/initialization.cpp.i

CMakeFiles/svo.dir/src/initialization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/initialization.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/initialization.cpp -o CMakeFiles/svo.dir/src/initialization.cpp.s

CMakeFiles/svo.dir/src/initialization.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/initialization.cpp.o.requires

CMakeFiles/svo.dir/src/initialization.cpp.o.provides: CMakeFiles/svo.dir/src/initialization.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/initialization.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/initialization.cpp.o.provides

CMakeFiles/svo.dir/src/initialization.cpp.o.provides.build: CMakeFiles/svo.dir/src/initialization.cpp.o

CMakeFiles/svo.dir/src/matcher.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/matcher.cpp.o: src/matcher.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/matcher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/matcher.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/matcher.cpp

CMakeFiles/svo.dir/src/matcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/matcher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/matcher.cpp > CMakeFiles/svo.dir/src/matcher.cpp.i

CMakeFiles/svo.dir/src/matcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/matcher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/matcher.cpp -o CMakeFiles/svo.dir/src/matcher.cpp.s

CMakeFiles/svo.dir/src/matcher.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/matcher.cpp.o.requires

CMakeFiles/svo.dir/src/matcher.cpp.o.provides: CMakeFiles/svo.dir/src/matcher.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/matcher.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/matcher.cpp.o.provides

CMakeFiles/svo.dir/src/matcher.cpp.o.provides.build: CMakeFiles/svo.dir/src/matcher.cpp.o

CMakeFiles/svo.dir/src/reprojector.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/reprojector.cpp.o: src/reprojector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/reprojector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/reprojector.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/reprojector.cpp

CMakeFiles/svo.dir/src/reprojector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/reprojector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/reprojector.cpp > CMakeFiles/svo.dir/src/reprojector.cpp.i

CMakeFiles/svo.dir/src/reprojector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/reprojector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/reprojector.cpp -o CMakeFiles/svo.dir/src/reprojector.cpp.s

CMakeFiles/svo.dir/src/reprojector.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/reprojector.cpp.o.requires

CMakeFiles/svo.dir/src/reprojector.cpp.o.provides: CMakeFiles/svo.dir/src/reprojector.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/reprojector.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/reprojector.cpp.o.provides

CMakeFiles/svo.dir/src/reprojector.cpp.o.provides.build: CMakeFiles/svo.dir/src/reprojector.cpp.o

CMakeFiles/svo.dir/src/feature_alignment.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/feature_alignment.cpp.o: src/feature_alignment.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/feature_alignment.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/feature_alignment.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/feature_alignment.cpp

CMakeFiles/svo.dir/src/feature_alignment.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/feature_alignment.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/feature_alignment.cpp > CMakeFiles/svo.dir/src/feature_alignment.cpp.i

CMakeFiles/svo.dir/src/feature_alignment.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/feature_alignment.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/feature_alignment.cpp -o CMakeFiles/svo.dir/src/feature_alignment.cpp.s

CMakeFiles/svo.dir/src/feature_alignment.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/feature_alignment.cpp.o.requires

CMakeFiles/svo.dir/src/feature_alignment.cpp.o.provides: CMakeFiles/svo.dir/src/feature_alignment.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/feature_alignment.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/feature_alignment.cpp.o.provides

CMakeFiles/svo.dir/src/feature_alignment.cpp.o.provides.build: CMakeFiles/svo.dir/src/feature_alignment.cpp.o

CMakeFiles/svo.dir/src/feature_detection.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/feature_detection.cpp.o: src/feature_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/feature_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/feature_detection.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/feature_detection.cpp

CMakeFiles/svo.dir/src/feature_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/feature_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/feature_detection.cpp > CMakeFiles/svo.dir/src/feature_detection.cpp.i

CMakeFiles/svo.dir/src/feature_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/feature_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/feature_detection.cpp -o CMakeFiles/svo.dir/src/feature_detection.cpp.s

CMakeFiles/svo.dir/src/feature_detection.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/feature_detection.cpp.o.requires

CMakeFiles/svo.dir/src/feature_detection.cpp.o.provides: CMakeFiles/svo.dir/src/feature_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/feature_detection.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/feature_detection.cpp.o.provides

CMakeFiles/svo.dir/src/feature_detection.cpp.o.provides.build: CMakeFiles/svo.dir/src/feature_detection.cpp.o

CMakeFiles/svo.dir/src/depth_filter.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/depth_filter.cpp.o: src/depth_filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/depth_filter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/depth_filter.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/depth_filter.cpp

CMakeFiles/svo.dir/src/depth_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/depth_filter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/depth_filter.cpp > CMakeFiles/svo.dir/src/depth_filter.cpp.i

CMakeFiles/svo.dir/src/depth_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/depth_filter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/depth_filter.cpp -o CMakeFiles/svo.dir/src/depth_filter.cpp.s

CMakeFiles/svo.dir/src/depth_filter.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/depth_filter.cpp.o.requires

CMakeFiles/svo.dir/src/depth_filter.cpp.o.provides: CMakeFiles/svo.dir/src/depth_filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/depth_filter.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/depth_filter.cpp.o.provides

CMakeFiles/svo.dir/src/depth_filter.cpp.o.provides.build: CMakeFiles/svo.dir/src/depth_filter.cpp.o

CMakeFiles/svo.dir/src/config.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/config.cpp.o: src/config.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/config.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/config.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/config.cpp

CMakeFiles/svo.dir/src/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/config.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/config.cpp > CMakeFiles/svo.dir/src/config.cpp.i

CMakeFiles/svo.dir/src/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/config.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/config.cpp -o CMakeFiles/svo.dir/src/config.cpp.s

CMakeFiles/svo.dir/src/config.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/config.cpp.o.requires

CMakeFiles/svo.dir/src/config.cpp.o.provides: CMakeFiles/svo.dir/src/config.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/config.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/config.cpp.o.provides

CMakeFiles/svo.dir/src/config.cpp.o.provides.build: CMakeFiles/svo.dir/src/config.cpp.o

CMakeFiles/svo.dir/src/sparse_img_align.cpp.o: CMakeFiles/svo.dir/flags.make
CMakeFiles/svo.dir/src/sparse_img_align.cpp.o: src/sparse_img_align.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/worxli/rpg/rpg_svo/svo/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/svo.dir/src/sparse_img_align.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/svo.dir/src/sparse_img_align.cpp.o -c /home/worxli/rpg/rpg_svo/svo/src/sparse_img_align.cpp

CMakeFiles/svo.dir/src/sparse_img_align.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/svo.dir/src/sparse_img_align.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/worxli/rpg/rpg_svo/svo/src/sparse_img_align.cpp > CMakeFiles/svo.dir/src/sparse_img_align.cpp.i

CMakeFiles/svo.dir/src/sparse_img_align.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/svo.dir/src/sparse_img_align.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/worxli/rpg/rpg_svo/svo/src/sparse_img_align.cpp -o CMakeFiles/svo.dir/src/sparse_img_align.cpp.s

CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.requires:
.PHONY : CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.requires

CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.provides: CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.requires
	$(MAKE) -f CMakeFiles/svo.dir/build.make CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.provides.build
.PHONY : CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.provides

CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.provides.build: CMakeFiles/svo.dir/src/sparse_img_align.cpp.o

# Object files for target svo
svo_OBJECTS = \
"CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o" \
"CMakeFiles/svo.dir/src/frame_handler_base.cpp.o" \
"CMakeFiles/svo.dir/src/frame.cpp.o" \
"CMakeFiles/svo.dir/src/point.cpp.o" \
"CMakeFiles/svo.dir/src/map.cpp.o" \
"CMakeFiles/svo.dir/src/pose_optimizer.cpp.o" \
"CMakeFiles/svo.dir/src/initialization.cpp.o" \
"CMakeFiles/svo.dir/src/matcher.cpp.o" \
"CMakeFiles/svo.dir/src/reprojector.cpp.o" \
"CMakeFiles/svo.dir/src/feature_alignment.cpp.o" \
"CMakeFiles/svo.dir/src/feature_detection.cpp.o" \
"CMakeFiles/svo.dir/src/depth_filter.cpp.o" \
"CMakeFiles/svo.dir/src/config.cpp.o" \
"CMakeFiles/svo.dir/src/sparse_img_align.cpp.o"

# External object files for target svo
svo_EXTERNAL_OBJECTS =

lib/libsvo.so: CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/frame_handler_base.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/frame.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/point.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/map.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/pose_optimizer.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/initialization.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/matcher.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/reprojector.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/feature_alignment.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/feature_detection.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/depth_filter.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/config.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/src/sparse_img_align.cpp.o
lib/libsvo.so: CMakeFiles/svo.dir/build.make
lib/libsvo.so: /usr/local/lib/libopencv_viz.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_videostab.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_video.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_ts.a
lib/libsvo.so: /usr/local/lib/libopencv_superres.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_stitching.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_photo.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_ocl.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_objdetect.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_nonfree.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_ml.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_legacy.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_imgproc.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_highgui.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_gpu.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_flann.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_features2d.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_core.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_contrib.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_calib3d.so.2.4.9
lib/libsvo.so: /home/worxli/Sophus/build/libSophus.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libpthread.so
lib/libsvo.so: /home/worxli/fast/build/libfast.so
lib/libsvo.so: /home/worxli/rpg/rpg_vikit/vikit_common/lib/libvikit_common.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libGLU.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libGL.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libSM.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libICE.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libX11.so
lib/libsvo.so: /usr/lib/x86_64-linux-gnu/libXext.so
lib/libsvo.so: /usr/local/lib/libopencv_nonfree.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_ocl.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_gpu.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_photo.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_objdetect.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_legacy.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_video.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_ml.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_calib3d.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_features2d.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_highgui.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_imgproc.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_flann.so.2.4.9
lib/libsvo.so: /usr/local/lib/libopencv_core.so.2.4.9
lib/libsvo.so: CMakeFiles/svo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library lib/libsvo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/svo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/svo.dir/build: lib/libsvo.so
.PHONY : CMakeFiles/svo.dir/build

CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/frame_handler_mono.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/frame_handler_base.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/frame.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/point.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/map.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/pose_optimizer.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/initialization.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/matcher.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/reprojector.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/feature_alignment.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/feature_detection.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/depth_filter.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/config.cpp.o.requires
CMakeFiles/svo.dir/requires: CMakeFiles/svo.dir/src/sparse_img_align.cpp.o.requires
.PHONY : CMakeFiles/svo.dir/requires

CMakeFiles/svo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/svo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/svo.dir/clean

CMakeFiles/svo.dir/depend:
	cd /home/worxli/rpg/rpg_svo/svo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo /home/worxli/rpg/rpg_svo/svo/CMakeFiles/svo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/svo.dir/depend

