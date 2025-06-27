cmake_minimum_required(VERSION 3.5)

project(simulation)

# SystemC configuration
set(SYSTEMC_HOME "$ENV{SYSTEMC_HOME}" CACHE PATH "Path to SystemC installation")
if(NOT SYSTEMC_HOME OR SYSTEMC_HOME STREQUAL "")
    message(FATAL_ERROR "SYSTEMC_HOME is not set. Please set it via -DSYSTEMC_HOME=... or export SYSTEMC_HOME=...")
endif()
set(SYSTEMC_INCLUDE_DIR ${SYSTEMC_HOME}/include)
if(EXISTS "${SYSTEMC_HOME}/lib64")
    set(SYSTEMC_LIBRARY_DIR ${SYSTEMC_HOME}/lib64)
else()
    set(SYSTEMC_LIBRARY_DIR ${SYSTEMC_HOME}/lib)
endif()

# Gurobi configuration
set(GUROBI_HOME "$ENV{GUROBI_HOME}" CACHE PATH "Path to Gurobi installation")
if(NOT GUROBI_HOME OR GUROBI_HOME STREQUAL "")
    message(FATAL_ERROR "GUROBI_HOME is not set. Please set it via -DGUROBI_HOME=... or export GUROBI_HOME=...")
endif()
set(GUROBI_INCLUDE_DIRS ${GUROBI_HOME}/include)
set(GUROBI_LIBRARY_DIR ${GUROBI_HOME}/lib)
set(GUROBI_LIBRARIES
    ${GUROBI_LIBRARY_DIR}/libgurobi_c++.a
    ${GUROBI_LIBRARY_DIR}/libgurobi120.so
)

# Eigen
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${SYSTEMC_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIRS}
)

# Source files
file(GLOB MAIN_SOURCES "src/*.cpp")
file(GLOB OTHER_SOURCES "*.cpp")
set(SOURCES ${OTHER_SOURCES} ${MAIN_SOURCES})

# Executable
add_executable(simulation ${SOURCES})

# Link libraries
target_link_libraries(simulation
    ${SYSTEMC_LIBRARY_DIR}/libsystemc.so
    ${GUROBI_LIBRARIES}
    pthread  # Gurobi may require pthread
    m        # Math library
)
