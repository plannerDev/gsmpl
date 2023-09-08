file(GLOB_RECURSE SOURCE_FILES "*.cpp" "base/*.cpp"
    "context/*.cpp" "goal/*.cpp" "planner_data/*.cpp" "robot_algo/*.cpp"
    "planner/*.cpp" "planner/rrt/*.cpp" "planner/bi_rrt/*.cpp" "planner/rrt_star/*.cpp" 
    "planner/informed_rrt_star/*.cpp" "tools/checker/*.cpp" "tools/distance/*.cpp"
    "tools/local_planner/*.cpp" "tools/nearest_neighbor/*.cpp"
    "tools/path_simplifier/*.cpp" "tools/sampler/*.cpp"
    "utility/*.cpp")
# aux_source_directory(. DIR_LIB_SRCS)
message("SOURCE_FILES: "${SOURCE_FILES})
add_library (gsmpl STATIC ${SOURCE_FILES})

if(WIN32)
    include_directories("D:/projects/dependencies/eigen3/include/eigen3")
    set(INSTALL_DIR D:/projects/dependencies/gsmpl)
elseif(UNIX)
    include_directories(${EIGEN3_INCLUDE_DIRS})
    set(INSTALL_DIR ~/ws_moveit2/install/gsmpl)
endif()

set(INCLUDE_DIRS
    base
    context
    goal
    planner_data
    robot_algo
    planner/rrt
    planner/bi_rrt
    planner/rrt_star
    planner/informed_rrt_star
    tools/checker
    tools/distance
    tools/local_planner
    tools/nearest_neighbor
    tools/path_simplifier
    tools/sampler
    utility
)
include_directories(${INCLUDE_DIRS})

install(TARGETS gsmpl DESTINATION ${INSTALL_DIR}/lib)

foreach(dir ${INCLUDE_DIRS})
    file(GLOB_RECURSE headers "${dir}/*.h")
    install(FILES ${headers} DESTINATION ${INSTALL_DIR}/include/${dir})
endforeach()
install(FILES planner_interface.h DESTINATION ${INSTALL_DIR}/include)
install(FILES planner/planner.h DESTINATION ${INSTALL_DIR}/include/planner)
