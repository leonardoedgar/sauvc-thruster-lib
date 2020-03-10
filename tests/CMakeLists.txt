project(sauvc-thruster-lib)
set(SOURCES ${SOURCES})

# Add dependencies needed from source dir to be tested  by unit tests
add_library(${CMAKE_PROJECT_NAME}_test_lib STATIC ../src/thruster_aggregator.cpp ../src/thruster_aggregator.h
        ../src/thruster.cpp ../src/thruster.h ../config/config.h)

target_include_directories(${CMAKE_PROJECT_NAME}_test_lib PUBLIC ${SOURCES})
include_directories(../lib/googletest)

# Add the list of test files want to be run when executable is run
add_executable(${CMAKE_PROJECT_NAME}_test main.cpp thruster_aggregator_test.cpp thruster_test.cpp)

# Add dependencies needed to run all unit tests
target_link_libraries(${CMAKE_PROJECT_NAME}_test PRIVATE ${CMAKE_PROJECT_NAME}_test_lib gtest)