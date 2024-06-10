cmake_minimum_required(VERSION 3.10)
project(ME553_2024)

set(CMAKE_CXX_STANDARD 17)
include(cmake/helper.cmake)

# exercise
#create_executable(exercise1 src/exercise1.cpp)
#create_executable(exercise2 src/exercise2.cpp)
#create_executable(CRBA src/exercise3.cpp)
#create_executable(RNE src/exercise4.cpp)
#create_executable(ABA src/exercise5.cpp)
create_executable(anymal src/anymal.cpp)
create_executable(arm src/arm.cpp)
#create_executable(debug src/debug.cpp)

# examples
#create_executable(rotation_example_angle_axis src/rotation_example_angle_axis.cpp)
#create_executable(rotation_example_euler_angles src/rotation_example_euler_angles.cpp)
#create_executable(dzhan src/dzhan.cpp)