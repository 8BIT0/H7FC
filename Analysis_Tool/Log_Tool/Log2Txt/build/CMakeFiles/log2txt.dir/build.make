# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.25.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.25.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build

# Include any dependencies generated for this target.
include CMakeFiles/log2txt.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/log2txt.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/log2txt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/log2txt.dir/flags.make

CMakeFiles/log2txt.dir/code/src/file_decode.c.o: CMakeFiles/log2txt.dir/flags.make
CMakeFiles/log2txt.dir/code/src/file_decode.c.o: /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/file_decode.c
CMakeFiles/log2txt.dir/code/src/file_decode.c.o: CMakeFiles/log2txt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/log2txt.dir/code/src/file_decode.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/log2txt.dir/code/src/file_decode.c.o -MF CMakeFiles/log2txt.dir/code/src/file_decode.c.o.d -o CMakeFiles/log2txt.dir/code/src/file_decode.c.o -c /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/file_decode.c

CMakeFiles/log2txt.dir/code/src/file_decode.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/log2txt.dir/code/src/file_decode.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/file_decode.c > CMakeFiles/log2txt.dir/code/src/file_decode.c.i

CMakeFiles/log2txt.dir/code/src/file_decode.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/log2txt.dir/code/src/file_decode.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/file_decode.c -o CMakeFiles/log2txt.dir/code/src/file_decode.c.s

CMakeFiles/log2txt.dir/code/src/main.c.o: CMakeFiles/log2txt.dir/flags.make
CMakeFiles/log2txt.dir/code/src/main.c.o: /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/main.c
CMakeFiles/log2txt.dir/code/src/main.c.o: CMakeFiles/log2txt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/log2txt.dir/code/src/main.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/log2txt.dir/code/src/main.c.o -MF CMakeFiles/log2txt.dir/code/src/main.c.o.d -o CMakeFiles/log2txt.dir/code/src/main.c.o -c /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/main.c

CMakeFiles/log2txt.dir/code/src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/log2txt.dir/code/src/main.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/main.c > CMakeFiles/log2txt.dir/code/src/main.c.i

CMakeFiles/log2txt.dir/code/src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/log2txt.dir/code/src/main.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/main.c -o CMakeFiles/log2txt.dir/code/src/main.c.s

CMakeFiles/log2txt.dir/code/src/minilzo.c.o: CMakeFiles/log2txt.dir/flags.make
CMakeFiles/log2txt.dir/code/src/minilzo.c.o: /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/minilzo.c
CMakeFiles/log2txt.dir/code/src/minilzo.c.o: CMakeFiles/log2txt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/log2txt.dir/code/src/minilzo.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/log2txt.dir/code/src/minilzo.c.o -MF CMakeFiles/log2txt.dir/code/src/minilzo.c.o.d -o CMakeFiles/log2txt.dir/code/src/minilzo.c.o -c /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/minilzo.c

CMakeFiles/log2txt.dir/code/src/minilzo.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/log2txt.dir/code/src/minilzo.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/minilzo.c > CMakeFiles/log2txt.dir/code/src/minilzo.c.i

CMakeFiles/log2txt.dir/code/src/minilzo.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/log2txt.dir/code/src/minilzo.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/minilzo.c -o CMakeFiles/log2txt.dir/code/src/minilzo.c.s

CMakeFiles/log2txt.dir/code/src/testmini.c.o: CMakeFiles/log2txt.dir/flags.make
CMakeFiles/log2txt.dir/code/src/testmini.c.o: /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/testmini.c
CMakeFiles/log2txt.dir/code/src/testmini.c.o: CMakeFiles/log2txt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/log2txt.dir/code/src/testmini.c.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/log2txt.dir/code/src/testmini.c.o -MF CMakeFiles/log2txt.dir/code/src/testmini.c.o.d -o CMakeFiles/log2txt.dir/code/src/testmini.c.o -c /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/testmini.c

CMakeFiles/log2txt.dir/code/src/testmini.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/log2txt.dir/code/src/testmini.c.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/testmini.c > CMakeFiles/log2txt.dir/code/src/testmini.c.i

CMakeFiles/log2txt.dir/code/src/testmini.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/log2txt.dir/code/src/testmini.c.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/code/src/testmini.c -o CMakeFiles/log2txt.dir/code/src/testmini.c.s

# Object files for target log2txt
log2txt_OBJECTS = \
"CMakeFiles/log2txt.dir/code/src/file_decode.c.o" \
"CMakeFiles/log2txt.dir/code/src/main.c.o" \
"CMakeFiles/log2txt.dir/code/src/minilzo.c.o" \
"CMakeFiles/log2txt.dir/code/src/testmini.c.o"

# External object files for target log2txt
log2txt_EXTERNAL_OBJECTS =

log2txt: CMakeFiles/log2txt.dir/code/src/file_decode.c.o
log2txt: CMakeFiles/log2txt.dir/code/src/main.c.o
log2txt: CMakeFiles/log2txt.dir/code/src/minilzo.c.o
log2txt: CMakeFiles/log2txt.dir/code/src/testmini.c.o
log2txt: CMakeFiles/log2txt.dir/build.make
log2txt: CMakeFiles/log2txt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C executable log2txt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/log2txt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/log2txt.dir/build: log2txt
.PHONY : CMakeFiles/log2txt.dir/build

CMakeFiles/log2txt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/log2txt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/log2txt.dir/clean

CMakeFiles/log2txt.dir/depend:
	cd /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build /Users/bit8/Desktop/develop/project/ARM/Cortex-M/H7FC/Analysis_Tool/Log2Txt/build/CMakeFiles/log2txt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/log2txt.dir/depend
