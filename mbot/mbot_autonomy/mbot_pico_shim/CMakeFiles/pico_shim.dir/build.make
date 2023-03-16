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
include mbot_pico_shim/CMakeFiles/pico_shim.dir/depend.make

# Include the progress variables for this target.
include mbot_pico_shim/CMakeFiles/pico_shim.dir/progress.make

# Include the compile flags for this target's objects.
include mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o: ../mbot_pico_shim/src/mbot_shim_main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/mbot_shim_main.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/mbot_shim_main.c > CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/mbot_shim_main.c -o CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.s

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.o: ../mbot_pico_shim/src/comms_common.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/comms_common.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/comms_common.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/comms_common.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/comms_common.c > CMakeFiles/pico_shim.dir/src/comms_common.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/comms_common.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/comms_common.c -o CMakeFiles/pico_shim.dir/src/comms_common.c.s

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.o: ../mbot_pico_shim/src/listener.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/listener.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/listener.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/listener.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/listener.c > CMakeFiles/pico_shim.dir/src/listener.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/listener.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/listener.c -o CMakeFiles/pico_shim.dir/src/listener.c.s

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.o: ../mbot_pico_shim/src/messages_mb.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/messages_mb.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/messages_mb.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/messages_mb.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/messages_mb.c > CMakeFiles/pico_shim.dir/src/messages_mb.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/messages_mb.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/messages_mb.c -o CMakeFiles/pico_shim.dir/src/messages_mb.c.s

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.o: ../mbot_pico_shim/src/protocol.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/protocol.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/protocol.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/protocol.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/protocol.c > CMakeFiles/pico_shim.dir/src/protocol.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/protocol.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/protocol.c -o CMakeFiles/pico_shim.dir/src/protocol.c.s

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.o: mbot_pico_shim/CMakeFiles/pico_shim.dir/flags.make
mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.o: ../mbot_pico_shim/src/topic_data.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.o"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/pico_shim.dir/src/topic_data.c.o   -c /home/pi/botlab-w23/mbot/mbot_pico_shim/src/topic_data.c

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pico_shim.dir/src/topic_data.c.i"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/botlab-w23/mbot/mbot_pico_shim/src/topic_data.c > CMakeFiles/pico_shim.dir/src/topic_data.c.i

mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pico_shim.dir/src/topic_data.c.s"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/botlab-w23/mbot/mbot_pico_shim/src/topic_data.c -o CMakeFiles/pico_shim.dir/src/topic_data.c.s

# Object files for target pico_shim
pico_shim_OBJECTS = \
"CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o" \
"CMakeFiles/pico_shim.dir/src/comms_common.c.o" \
"CMakeFiles/pico_shim.dir/src/listener.c.o" \
"CMakeFiles/pico_shim.dir/src/messages_mb.c.o" \
"CMakeFiles/pico_shim.dir/src/protocol.c.o" \
"CMakeFiles/pico_shim.dir/src/topic_data.c.o"

# External object files for target pico_shim
pico_shim_EXTERNAL_OBJECTS =

bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/mbot_shim_main.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/comms_common.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/listener.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/messages_mb.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/protocol.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/src/topic_data.c.o
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/build.make
bin/pico_shim: /usr/local/lib/liblcm.so.1.4.0
bin/pico_shim: common_utils/libcommon_utils.a
bin/pico_shim: mbot_lcm_msgs/libmbot_lcm_msgs.a
bin/pico_shim: /usr/local/lib/liblcm.so.1.4.0
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libglib-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgobject-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libatk-1.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgio-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgthread-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgmodule-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgdk_pixbuf-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libcairo.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libharfbuzz.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libpango-1.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libpangocairo-1.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libpangoft2-1.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libpangoxft-1.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgdk-x11-2.0.so
bin/pico_shim: /usr/lib/aarch64-linux-gnu/libgtk-x11-2.0.so
bin/pico_shim: mbot_pico_shim/CMakeFiles/pico_shim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/botlab-w23/mbot/mbot_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../bin/pico_shim"
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pico_shim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
mbot_pico_shim/CMakeFiles/pico_shim.dir/build: bin/pico_shim

.PHONY : mbot_pico_shim/CMakeFiles/pico_shim.dir/build

mbot_pico_shim/CMakeFiles/pico_shim.dir/clean:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim && $(CMAKE_COMMAND) -P CMakeFiles/pico_shim.dir/cmake_clean.cmake
.PHONY : mbot_pico_shim/CMakeFiles/pico_shim.dir/clean

mbot_pico_shim/CMakeFiles/pico_shim.dir/depend:
	cd /home/pi/botlab-w23/mbot/mbot_autonomy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/botlab-w23/mbot /home/pi/botlab-w23/mbot/mbot_pico_shim /home/pi/botlab-w23/mbot/mbot_autonomy /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim /home/pi/botlab-w23/mbot/mbot_autonomy/mbot_pico_shim/CMakeFiles/pico_shim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : mbot_pico_shim/CMakeFiles/pico_shim.dir/depend
