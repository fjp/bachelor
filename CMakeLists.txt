cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_subdirectory(visualizer)
add_subdirectory(planner)

add_executable(Bachelor main.cpp)
target_link_libraries(Bachelor visualizer planner)

add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





