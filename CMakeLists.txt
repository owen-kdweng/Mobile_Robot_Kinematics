cmake_minimum_required(VERSION 3.10)

project(Mobile_Robot_Kinematics C)

add_library(dd_kinematics src/differential_drive_kinematics.c)
target_include_directories(dd_kinematics PUBLIC include)

add_executable(dd_example examples/dd_example.c)
target_link_libraries(dd_example dd_kinematics m)