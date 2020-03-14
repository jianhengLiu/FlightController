cmake_minimum_required(VERSION 2.8.3)
project(tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(FlightController
        include/flightController.h
        src/flightController.cpp)
target_link_libraries(FlightController
        ${catkin_LIBRARIES}
        )

add_executable(main
        src/main.cpp
        )
target_link_libraries(main
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        FlightController
        )
