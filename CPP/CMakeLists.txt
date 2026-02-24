cmake_minimum_required(VERSION 3.14)
project(astar_algorithm VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(astar_cpp
    main.cpp
    astar_class.cpp
)

target_include_directories(astar_cpp PRIVATE
    includes
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(astar_cpp PRIVATE
    ${OpenCV_LIBS}
)

set_target_properties(astar_cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
