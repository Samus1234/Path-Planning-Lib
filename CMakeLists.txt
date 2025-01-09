cmake_minimum_required(VERSION 3.10)

project(RoboticsUtils LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile options
add_compile_options(-Wall -O2)

# Include directories
include_directories(
    /usr/include/python3.10             # Python3 include path
    /usr/local/include/eigen            # Eigen include path
    ${CMAKE_SOURCE_DIR}/include         # Include your project's include directory
)

# Find Python3 Development package
find_package(Python3 COMPONENTS Development REQUIRED)
include_directories(${Python3_INCLUDE_DIRS})
link_libraries(${Python3_LIBRARIES})

# Include matplotlib-cpp
include_directories(${CMAKE_SOURCE_DIR}/third_party/matplotlib-cpp)

# Define source and include directories
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(EXAMPLES_DIR "${CMAKE_SOURCE_DIR}/examples")
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
set(PLOTS_DIR "${CMAKE_SOURCE_DIR}/plots")

include_directories(${INCLUDE_DIR})

# Collect all source files
file(GLOB_RECURSE SOURCES ${SRC_DIR}/*.cpp)
file(GLOB_RECURSE HEADERS ${INCLUDE_DIR}/*.hpp)
file(GLOB_RECURSE EXAMPLES ${EXAMPLES_DIR}/*.cpp)

# Create a core library
add_library(core STATIC ${SOURCES} ${HEADERS})

# Add matplotlib dependencies to core library
target_link_libraries(core Python3::Python)

# Loop through example files and create executables
foreach(EXAMPLE_FILE ${EXAMPLES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} core Python3::Python)
    set_target_properties(${EXAMPLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${EXAMPLE_NAME})
endforeach()

# Optional: Loop through sources and create individual executables
foreach(SRC_FILE ${SOURCES})
    get_filename_component(SRC_NAME ${SRC_FILE} NAME_WE)
    add_executable(${SRC_NAME} ${SRC_FILE})
    target_link_libraries(${SRC_NAME} core Python3::Python)
    set_target_properties(${SRC_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${SRC_NAME})
endforeach()

# Add a custom target to run all examples
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
