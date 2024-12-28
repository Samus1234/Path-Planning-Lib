cmake_minimum_required(VERSION 3.10)

project(RoboticsUtils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -O2)

include_directories(
    /usr/include/python3.10
    /usr/local/include/eigen
)

find_package(Python3 COMPONENTS Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
link_libraries(${Python3_LIBRARIES})

set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(EXAMPLES_DIR "${CMAKE_SOURCE_DIR}/examples")
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
set(PLOTS_DIR "${CMAKE_SOURCE_DIR}/plots")

include_directories(${INCLUDE_DIR})

file(GLOB_RECURSE SOURCES ${SRC_DIR}/*.cpp)
file(GLOB_RECURSE HEADERS ${INCLUDE_DIR}/*.hpp)
file(GLOB_RECURSE EXAMPLES ${EXAMPLES_DIR}/*.cpp)

add_library(core STATIC ${SOURCES} ${HEADERS})

foreach(EXAMPLE_FILE ${EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} core Python3::Python)
    set_target_properties(${EXAMPLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${EXAMPLE_NAME})
endforeach()

add_custom_target(run_all_examples
    COMMAND ${CMAKE_COMMAND} -E echo "Running all examples..."
    COMMAND ${CMAKE_COMMAND} -E echo "Running A*_test..." && ./Astar_test/Astar_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running arc_length_trajectory_test..." && ./arc_length_trajectory_test/arc_length_trajectory_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running binary_tree_test..." && ./binary_tree_test/binary_tree_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running circular_buffer_test..." && ./circular_buffer_test/circular_buffer_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running Dijkstra_test..." && ./dijkstra_test/dijkstra_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running linked_list_test..." && ./linked_list_test/linked_list_test
    COMMAND ${CMAKE_COMMAND} -E echo "Running priority_queue_test..." && ./priority_queue_test/priority_queue_test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
