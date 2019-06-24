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
        PUBLIC cxx_lambdas cxx_auto_type #cxx_generalized_initializers cxx_aggregate_default_initializers
        PRIVATE cxx_lambdas cxx_auto_type) #cxx_generalized_initializers cxx_aggregate_default_initializers)

#set_property(TARGET planner PROPERTY CXX_STANDARD 20)
#set_property(TARGET planner PROPERTY CXX_STANDARD_REQUIRED ON)