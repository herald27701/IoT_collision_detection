# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.26

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build

# Include any dependencies generated for this target.
include CMakeFiles/VerifyFortranC.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/VerifyFortranC.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/VerifyFortranC.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VerifyFortranC.dir/flags.make

CMakeFiles/VerifyFortranC.dir/main.c.obj: CMakeFiles/VerifyFortranC.dir/flags.make
CMakeFiles/VerifyFortranC.dir/main.c.obj: CMakeFiles/VerifyFortranC.dir/includes_C.rsp
CMakeFiles/VerifyFortranC.dir/main.c.obj: C:/Program\ Files/CMake/share/cmake-3.26/Modules/FortranCInterface/Verify/main.c
CMakeFiles/VerifyFortranC.dir/main.c.obj: CMakeFiles/VerifyFortranC.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/VerifyFortranC.dir/main.c.obj"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/VerifyFortranC.dir/main.c.obj -MF CMakeFiles\VerifyFortranC.dir\main.c.obj.d -o CMakeFiles\VerifyFortranC.dir\main.c.obj -c "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\main.c"

CMakeFiles/VerifyFortranC.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/VerifyFortranC.dir/main.c.i"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\main.c" > CMakeFiles\VerifyFortranC.dir\main.c.i

CMakeFiles/VerifyFortranC.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/VerifyFortranC.dir/main.c.s"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\main.c" -o CMakeFiles\VerifyFortranC.dir\main.c.s

CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj: CMakeFiles/VerifyFortranC.dir/flags.make
CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj: CMakeFiles/VerifyFortranC.dir/includes_C.rsp
CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj: C:/Program\ Files/CMake/share/cmake-3.26/Modules/FortranCInterface/Verify/VerifyC.c
CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj: CMakeFiles/VerifyFortranC.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj -MF CMakeFiles\VerifyFortranC.dir\VerifyC.c.obj.d -o CMakeFiles\VerifyFortranC.dir\VerifyC.c.obj -c "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\VerifyC.c"

CMakeFiles/VerifyFortranC.dir/VerifyC.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/VerifyFortranC.dir/VerifyC.c.i"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\VerifyC.c" > CMakeFiles\VerifyFortranC.dir\VerifyC.c.i

CMakeFiles/VerifyFortranC.dir/VerifyC.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/VerifyFortranC.dir/VerifyC.c.s"
	D:\MinGW\bin\gcc.exe $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify\VerifyC.c" -o CMakeFiles\VerifyFortranC.dir\VerifyC.c.s

# Object files for target VerifyFortranC
VerifyFortranC_OBJECTS = \
"CMakeFiles/VerifyFortranC.dir/main.c.obj" \
"CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj"

# External object files for target VerifyFortranC
VerifyFortranC_EXTERNAL_OBJECTS =

VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/main.c.obj
VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/VerifyC.c.obj
VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/build.make
VerifyFortranC.exe: libVerifyFortran.a
VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/linkLibs.rsp
VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/objects1.rsp
VerifyFortranC.exe: CMakeFiles/VerifyFortranC.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable VerifyFortranC.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\VerifyFortranC.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VerifyFortranC.dir/build: VerifyFortranC.exe
.PHONY : CMakeFiles/VerifyFortranC.dir/build

CMakeFiles/VerifyFortranC.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\VerifyFortranC.dir\cmake_clean.cmake
.PHONY : CMakeFiles/VerifyFortranC.dir/clean

CMakeFiles/VerifyFortranC.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify" "C:\Program Files\CMake\share\cmake-3.26\Modules\FortranCInterface\Verify" F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build F:\Users\acer\Documents\PlatformIO\Projects\IoT_collision_detection\build\CMakeFiles\VerifyFortranC.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VerifyFortranC.dir/depend

