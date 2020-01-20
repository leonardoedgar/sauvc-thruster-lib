project(SAUVC2020)
set(SOURCES ${SOURCES})

# Add dependencies needed from source dir to be tested  by unit tests
add_library(${CMAKE_PROJECT_NAME}_test_lib STATIC ../src/motor_controller.cpp ../src/motor_controller.h
        ../src/motor_driver.cpp ../src/motor_driver.h ../config/config.h)

target_include_directories(${CMAKE_PROJECT_NAME}_test_lib PUBLIC ${SOURCES})
include_directories(../lib/googletest)

# Add the list of test files want to be run when executable is run
add_executable(${CMAKE_PROJECT_NAME}_test main.cpp motor_controller_test.cpp motor_driver_test.cpp)

# Add dependencies needed to run all unit tests
target_link_libraries(${CMAKE_PROJECT_NAME}_test PRIVATE ${CMAKE_PROJECT_NAME}_test_lib gtest)