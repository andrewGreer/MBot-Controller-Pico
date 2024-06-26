# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/mbot-controller-pico

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/mbot-controller-pico/build

# Utility rule file for ELF2UF2Build.

# Include the progress variables for this target.
include src/CMakeFiles/ELF2UF2Build.dir/progress.make

src/CMakeFiles/ELF2UF2Build: src/CMakeFiles/ELF2UF2Build-complete


src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
src/CMakeFiles/ELF2UF2Build-complete: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/CMakeFiles
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/CMakeFiles/ELF2UF2Build-complete
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-done

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No install step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/elf2uf2 && /usr/bin/cmake -E echo_append
	cd /home/pi/mbot-controller-pico/build/elf2uf2 && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/lib/pico-sdk/tools/elf2uf2
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/elf2uf2
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/elf2uf2
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/elf2uf2/tmp
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/elf2uf2/src
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E make_directory /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E echo_append
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No update step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E echo_append
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E echo_append
	cd /home/pi/mbot-controller-pico/build/src && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure: src/elf2uf2/tmp/ELF2UF2Build-cfgcmd.txt
src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/elf2uf2 && /usr/bin/cmake "-GUnix Makefiles" /home/pi/mbot-controller-pico/lib/pico-sdk/tools/elf2uf2
	cd /home/pi/mbot-controller-pico/build/elf2uf2 && /usr/bin/cmake -E touch /home/pi/mbot-controller-pico/build/src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure

src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/mbot-controller-pico/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'ELF2UF2Build'"
	cd /home/pi/mbot-controller-pico/build/elf2uf2 && $(MAKE)

ELF2UF2Build: src/CMakeFiles/ELF2UF2Build
ELF2UF2Build: src/CMakeFiles/ELF2UF2Build-complete
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
ELF2UF2Build: src/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
ELF2UF2Build: src/CMakeFiles/ELF2UF2Build.dir/build.make

.PHONY : ELF2UF2Build

# Rule to build all files generated by this target.
src/CMakeFiles/ELF2UF2Build.dir/build: ELF2UF2Build

.PHONY : src/CMakeFiles/ELF2UF2Build.dir/build

src/CMakeFiles/ELF2UF2Build.dir/clean:
	cd /home/pi/mbot-controller-pico/build/src && $(CMAKE_COMMAND) -P CMakeFiles/ELF2UF2Build.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ELF2UF2Build.dir/clean

src/CMakeFiles/ELF2UF2Build.dir/depend:
	cd /home/pi/mbot-controller-pico/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/mbot-controller-pico /home/pi/mbot-controller-pico/src /home/pi/mbot-controller-pico/build /home/pi/mbot-controller-pico/build/src /home/pi/mbot-controller-pico/build/src/CMakeFiles/ELF2UF2Build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ELF2UF2Build.dir/depend

