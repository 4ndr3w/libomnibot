cmake_minimum_required(VERSION 3.1)

project(libomnibot)
set(CMAKE_CXX_STANDARD 11)
add_library(omnibot OmniBot.cpp OmniBotStateReceiver.cpp OmniBotCommandSender.cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
