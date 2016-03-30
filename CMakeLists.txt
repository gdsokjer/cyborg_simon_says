cmake_minimum_required(VERSION 2.8.3)
project(cyborg_simon_says)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation
)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs message_generation
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(cyborg_simon_says_node src/cyborg_simon_says_node.cpp)
add_dependencies(cyborg_simon_says_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cyborg_simon_says_node ${catkin_LIBRARIES})
