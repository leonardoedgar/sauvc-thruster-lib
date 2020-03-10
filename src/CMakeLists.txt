project(sauvc-thruster-lib)
set(SOURCES ${SOURCES})

# Add dependencies needed to run the main executable file
add_library(${CMAKE_PROJECT_NAME}_lib STATIC thruster_aggregator.cpp thruster_aggregator.h thruster.cpp thruster.h ../config/config.h)
target_include_directories(${CMAKE_PROJECT_NAME}_lib PUBLIC ${SOURCES})

add_executable(${CMAKE_PROJECT_NAME} main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_PROJECT_NAME}_lib)
