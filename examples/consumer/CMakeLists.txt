cmake_minimum_required(VERSION 3.16)
project(rozeta_consumer C CXX)

find_package(rozeta CONFIG REQUIRED)

add_executable(consumer_c main.c)
target_link_libraries(consumer_c PRIVATE rozeta::rozeta)

add_executable(consumer_cpp main.cpp)
target_link_libraries(consumer_cpp PRIVATE rozeta::rozeta)
