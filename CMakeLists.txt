project(gsmpl)
cmake_minimum_required(VERSION 3.5)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options("-O3")

set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_BUILD_TYPE "Debug")

if(WIN32)
    find_package(Eigen3 REQUIRED PATHS "D:/projects/dependencies/eigen3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2 /W4 /we4010 /we4099 /we4114 /we4129 
                       /we4130 /we4132 /we4146 /we4172 /we4265 /we4269 /we4273 /we4280
                       /we4281 /we4293 /we4296 /we4297 /we4307 /we4333 /we4651 /we4700
                       /we4701 /we4702 /we4709 /we4715 /we4717 /we4722 /we4723 /we4724
                       /we4804 /we4805 /we4834 /wd4127 /wd4201 /wd4245 /wd4251 /wd4267
                       /wd4305 /wd4458 /wd4819 /wd5054 /bigobj")
elseif(UNIX)
    find_package(Eigen3 REQUIRED)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message("Eigen3 dir: ${EIGEN3_INCLUDE_DIR}")
add_subdirectory(src)
