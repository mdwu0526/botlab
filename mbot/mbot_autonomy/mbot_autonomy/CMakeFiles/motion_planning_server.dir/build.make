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
include mbot_autonomy/CMakeFiles/motion_planning_server.dir/depend.make

# Include the progress variables for this target.
include mbot_autonomy/CMakeFiles/motion_planning_server.dir/progress.make

# Include the compile flags for this target's objects.
include mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o: src/planning/motion_planner_server_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server_main.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server_main.cpp > CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server_main.cpp -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.s

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o: src/planning/motion_planner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner.cpp > CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner.cpp -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.s

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o: src/planning/motion_planner_server.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server.cpp > CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/motion_planner_server.cpp -o CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.s

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o: src/slam/occupancy_grid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/slam/occupancy_grid.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/slam/occupancy_grid.cpp > CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/slam/occupancy_grid.cpp -o CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.s

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o: src/planning/obstacle_distance_grid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/obstacle_distance_grid.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/obstacle_distance_grid.cpp > CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/obstacle_distance_grid.cpp -o CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.s

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o: mbot_autonomy/CMakeFiles/motion_planning_server.dir/flags.make
mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o: src/planning/astar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o -c /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/astar.cpp

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/astar.cpp > CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.i

mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_autonomy/src/planning/astar.cpp -o CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.s

# Object files for target motion_planning_server
motion_planning_server_OBJECTS = \
"CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o" \
"CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o" \
"CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o" \
"CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o" \
"CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o" \
"CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o"

# External object files for target motion_planning_server
motion_planning_server_EXTERNAL_OBJECTS =

bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server_main.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/motion_planner_server.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/slam/occupancy_grid.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/obstacle_distance_grid.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/src/planning/astar.cpp.o
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/build.make
bin/motion_planning_server: common_utils/libcommon_utils.a
bin/motion_planning_server: /usr/local/lib/liblcm.so.1.4.0
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libglib-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgobject-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libatk-1.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgio-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgthread-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgmodule-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgdk_pixbuf-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libcairo.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libharfbuzz.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libpango-1.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libpangocairo-1.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libpangoft2-1.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libpangoxft-1.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgdk-x11-2.0.so
bin/motion_planning_server: /usr/lib/aarch64-linux-gnu/libgtk-x11-2.0.so
bin/motion_planning_server: mbot_lcm_msgs/libmbot_lcm_msgs.a
bin/motion_planning_server: /usr/local/lib/liblcm.so.1.4.0
bin/motion_planning_server: mbot_autonomy/CMakeFiles/motion_planning_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../bin/motion_planning_server"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motion_planning_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mbot_autonomy/CMakeFiles/motion_planning_server.dir/build: bin/motion_planning_server

.PHONY : mbot_autonomy/CMakeFiles/motion_planning_server.dir/build

mbot_autonomy/CMakeFiles/motion_planning_server.dir/clean:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy && $(CMAKE_COMMAND) -P CMakeFiles/motion_planning_server.dir/cmake_clean.cmake
.PHONY : mbot_autonomy/CMakeFiles/motion_planning_server.dir/clean

mbot_autonomy/CMakeFiles/motion_planning_server.dir/depend:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/botlab-w23/mbot /home/pi/botlab-w23/mbot/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_autonomy/CMakeFiles/motion_planning_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mbot_autonomy/CMakeFiles/motion_planning_server.dir/depend

