file(GLOB SRC
        "*.cpp"
        "../include/*.h"
        )

add_library(planner ${SRC})

#message(${SRC})




target_include_directories(planner PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:../include>
        PRIVATE ../include
        PUBLIC ../../
        PUBLIC ../../visualizer/include
        )

target_compile_features(planner
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)