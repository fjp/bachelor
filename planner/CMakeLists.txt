add_subdirectory(src)
#add_subdirectory(doc)
add_subdirectory(test)


#file(GLOB SRC
#        "*.cpp"
#        "include/*.h"
#        )
#
##add_library(planner ${SRC})
#add_library(planner
#        src/rover_interface.cpp
#        src/audi_rover.cpp
#        include/rover_interface.h
#        include/audi_rover.h)
#
#message(${SRC})
#
#target_include_directories(planner PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#        PRIVATE include)
#
#target_compile_features(planner
#        PUBLIC cxx_lambdas cxx_auto_type
#        PRIVATE cxx_lambdas cxx_auto_type)
