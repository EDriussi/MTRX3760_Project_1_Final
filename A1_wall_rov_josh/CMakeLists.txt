cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME A1.WallFollower)

project(${CMAKE_PROJECT_NAME} LANGUAGES CXX)

add_executable(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/main.cpp
    Classes/Src/CLoopReader.cpp
    Classes/Src/CRender.cpp
    Classes/Src/Robot.cpp
    Classes/Src/Map.cpp
    Classes/Src/CMotor.cpp
    Classes/Src/CRangeSensor.cpp
    Classes/Src/CSubsystem.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Classes/Inc
)

find_package(raylib REQUIRED)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE 
    raylib
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/Maps/SimpleWalls.map
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -g
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wold-style-cast
    -Wcast-align
    -Wconversion
    -Wsign-conversion
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wlogical-op
    -Wuseless-cast
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -g
    #-static
    -static-libgcc
    -static-libstdc++
)

