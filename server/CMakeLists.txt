cmake_minimum_required(VERSION 2.8)
project(pepper_camera_server)

find_package(qibuild)

include_directories(include)
qi_create_bin(pepper_camera_server "src/pepper_camera_server.cpp")
