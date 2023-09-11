cmake_minimum_required(VERSION 3.5)
project(gsmpl)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options("-O3")

set(CMAKE_CXX_STANDARD 17)
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

# message("Eigen3 dir: ${EIGEN3_INCLUDE_DIR}")
# add_subdirectory(src)

# file(GLOB_RECURSE SOURCE_FILES "*.cpp" "base/*.cpp"
#     "context/*.cpp" "goal/*.cpp" "planner_data/*.cpp" "robot_algo/*.cpp"
#     "planner/*.cpp" "planner/rrt/*.cpp" "planner/bi_rrt/*.cpp" "planner/rrt_star/*.cpp" 
#     "planner/informed_rrt_star/*.cpp" "tools/checker/*.cpp" "tools/distance/*.cpp"
#     "tools/local_planner/*.cpp" "tools/nearest_neighbor/*.cpp"
#     "tools/path_simplifier/*.cpp" "tools/sampler/*.cpp"
#     "utility/*.cpp")
file(GLOB_RECURSE ALL_LIBRARY_HDRS "gsmpl/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "gsmpl/*.cpp")
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
endif()

# aux_source_directory(. DIR_LIB_SRCS)
message("ALL_LIBRARY_HDRS: "${ALL_LIBRARY_HDRS})
message("ALL_LIBRARY_SRCS: "${ALL_LIBRARY_SRCS})


add_library (${PROJECT_NAME} STATIC ${ALL_LIBRARY_SRCS})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

if (WIN32)
  find_package(glog REQUIRED)
  set(GLOG_LIBRARY glog::glog)
else()
  set(GLOG_LIBRARY glog)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)
target_link_libraries(${PROJECT_NAME} PUBLIC pthread)

# if(WIN32)
#     include_directories("D:/projects/dependencies/eigen3/include/eigen3")
#     set(INSTALL_DIR D:/projects/dependencies/gsmpl)
# elseif(UNIX)
#     include_directories(${EIGEN3_INCLUDE_DIRS})
#     set(INSTALL_DIR ~/ws_moveit2/install/gsmpl)
# endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT GsmplExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${ALL_LIBRARY_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

set(GSMPL_CMAKE_DIR share/gsmpl/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  gsmpl-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/gsmpl/gsmpl-config.cmake
  PATH_VARS GSMPL_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gsmpl
)
install(
  EXPORT GsmplExport
  DESTINATION share/gsmpl/cmake/
  FILE GSMPLTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/gsmpl/gsmpl-config.cmake
  DESTINATION share/gsmpl/
)

# set(INCLUDE_DIRS
#     base
#     context
#     goal
#     planner_data
#     robot_algo
#     planner/rrt
#     planner/bi_rrt
#     planner/rrt_star
#     planner/informed_rrt_star
#     tools/checker
#     tools/distance
#     tools/local_planner
#     tools/nearest_neighbor
#     tools/path_simplifier
#     tools/sampler
#     utility
# )
# include_directories(${INCLUDE_DIRS})

# install(TARGETS gsmpl DESTINATION ${INSTALL_DIR}/lib)

# foreach(dir ${INCLUDE_DIRS})
#     file(GLOB_RECURSE headers "${dir}/*.h")
#     install(FILES ${headers} DESTINATION ${INSTALL_DIR}/include/${dir})
# endforeach()
# install(FILES planner_interface.h DESTINATION ${INSTALL_DIR}/include)
# install(FILES planner/planner.h DESTINATION ${INSTALL_DIR}/include/planner)
# install(FILES package.xml DESTINATION share/gsmpl)
