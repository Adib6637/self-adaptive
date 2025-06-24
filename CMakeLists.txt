cmake_minimum_required(VERSION 3.5)

project(simulation)

# SystemC configuration
set(SYSTEMC_HOME "/scratch/hpc-prf-ekiapp/sheikh/exploration/systemc/install")
set(SYSTEMC_INCLUDE_DIR ${SYSTEMC_HOME}/include)
set(SYSTEMC_LIBRARY_DIR ${SYSTEMC_HOME}/lib64)

# Gurobi configuration
set(GUROBI_HOME /opt/software/pc2/GUROBI/gurobi1202/linux64)
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
