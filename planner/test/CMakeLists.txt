################################
# Unit Tests
################################
# Add test cpp file
add_executable(testPlanner test.cpp test_fixture.h)
# Link test executable against gtest & gtest_main
target_include_directories(testPlanner PUBLIC
        ../../)
target_link_libraries(testPlanner gtest gtest_main planner visualizer)
add_test(TestPlanner TestPlanner)