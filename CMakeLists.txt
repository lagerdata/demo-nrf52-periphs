cmake_minimum_required(VERSION 3.13)

set(CMAKE_TOOLCHAIN_FILE ../cmake/arm-gcc-toolchain.cmake)

# set the project name and version
project(demo-atsame70-xpld VERSION 1.0 LANGUAGES C ASM)



# Location of application specific code
add_subdirectory(app)
# Location of test code. These will be run on deploys
add_subdirectory(unit-tests)
#Location of standalone/reusable modules
add_subdirectory(modules)
add_subdirectory(zig)