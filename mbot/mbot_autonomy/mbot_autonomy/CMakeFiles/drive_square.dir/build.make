# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/botlab-w23/mbot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/botlab-w23/mbot/mbot_autonomy

# Include any dependencies generated for this target.
include mbot_autonomy/CMakeFiles/drive_square.dir/depend.make

# Include the progress variables for this target.
include mbot_autonomy/CMakeFiles/drive_square.dir/progress.make

# Include the compile flags for this target's objects.
include mbot_autonomy/CMakeFiles/drive_square.dir/flags.make

mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o: mbot_autonomy/CMakeFiles/drive_square.dir/flags.make
mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o: src/mbot/drive_square.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/mbot/drive_square.cpp

mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/mbot/drive_square.cpp > CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.i

mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/mbot/drive_square.cpp -o CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.s

# Object files for target drive_square
drive_square_OBJECTS = \
"CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o"

# External object files for target drive_square
drive_square_EXTERNAL_OBJECTS =

bin/drive_square: mbot_autonomy/CMakeFiles/drive_square.dir/src/mbot/drive_square.cpp.o
bin/drive_square: mbot_autonomy/CMakeFiles/drive_square.dir/build.make
bin/drive_square: common_utils/libcommon_utils.a
bin/drive_square: /usr/local/lib/liblcm.so.1.4.0
bin/drive_square: /usr/lib/aarch64-linux-gnu/libglib-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgobject-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libatk-1.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgio-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgthread-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgmodule-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgdk_pixbuf-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libcairo.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libharfbuzz.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libpango-1.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libpangocairo-1.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libpangoft2-1.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libpangoxft-1.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgdk-x11-2.0.so
bin/drive_square: /usr/lib/aarch64-linux-gnu/libgtk-x11-2.0.so
bin/drive_square: mbot_lcm_msgs/libmbot_lcm_msgs.a
bin/drive_square: /usr/local/lib/liblcm.so.1.4.0
bin/drive_square: mbot_autonomy/CMakeFiles/drive_square.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/drive_square"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drive_square.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mbot_autonomy/CMakeFiles/drive_square.dir/build: bin/drive_square

.PHONY : mbot_autonomy/CMakeFiles/drive_square.dir/build

mbot_autonomy/CMakeFiles/drive_square.dir/clean:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && $(CMAKE_COMMAND) -P CMakeFiles/drive_square.dir/cmake_clean.cmake
.PHONY : mbot_autonomy/CMakeFiles/drive_square.dir/clean

mbot_autonomy/CMakeFiles/drive_square.dir/depend:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/botlab-w23/mbot /home/pi/botlab-w23/mbot/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy/CMakeFiles/drive_square.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mbot_autonomy/CMakeFiles/drive_square.dir/depend
