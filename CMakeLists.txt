cmake_minimum_required(VERSION 3.5)
project(multiviperfrog)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3)

# Color
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta     "${Esc}[35m")
endif()

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        roslib
        std_msgs
        open3d_conversions
        eigen_conversions
        tf
        tf2
        tf2_ros
        tf2_geometry_msgs
        rosbag
        )

# Find necessary catkin packages
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

# Find external dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
# Print the location of found Ceres library
message("${BoldMagenta}Ceres library found at: ${Ceres_INCLUDE_DIRS}${ColourReset}")
find_package(Open3D REQUIRED)
find_package(open3d_conversions REQUIRED)

# Catkin package setup
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
)

# Include directories
message("${Magenta}Include catkin directories: ${catkin_INCLUDE_DIRS}${ColourReset}")
include_directories(
        include
        ${CATKIN_DEVEL_PREFIX}/include  # TODO: remove in the future
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${Open3D_INCLUDE_DIRS}
        ${open3d_conversions_INCLUDE_DIRS}
)

# Source files for the core library
set(CORE_SOURCES
        src/core/preprocessing/preprocessing.cpp
        src/core/optimization/optimization.cpp
        src/core/experiment_module.cpp
        src/core/optimization/problems.cpp
        src/core/workflow_manager.cpp src/core/postprocessing/postprocessing.cpp)

# Cost function source files
file(GLOB COST_FUNCTION_SOURCES
        ${PROJECT_SOURCE_DIR}/src/core/optimization/cost_functions/*.cpp
        )

# Interface source files
set(INTERFACE_SOURCES
        src/interfaces/offline_input_adapter.cpp
        src/interfaces/ros_input_adapter.cpp
        src/interfaces/ros_output_adapter.cpp
        src/interfaces/holoscan_input_adapter.cpp
        src/interfaces/holoscan_output_adapter.cpp
        src/interfaces/RosDataProcessor.cpp
        src/interfaces/RosOnlineDataProcessor.cpp
        src/interfaces/RosRosbagDataProcessor.cpp
        src/interfaces/helpers_ros.cpp)

# Utility source files
set(UTIL_SOURCES
        src/utils/data_conversions.cpp
        src/utils/optimization_utils.cpp
        src/utils/preprocessing_utils.cpp
        src/utils/KDTreeFlann.cpp
        )

# Fetch external libraries such as nlohmann_json
include(FetchContent)
FetchContent_Declare(
        json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)

# Create core library
add_library(${PROJECT_NAME}
        ${CORE_SOURCES}
        ${COST_FUNCTION_SOURCES}
        ${INTERFACE_SOURCES}
        ${UTIL_SOURCES}
        )

# Link libraries to core_lib
target_link_libraries(${PROJECT_NAME}
        PRIVATE nlohmann_json::nlohmann_json # TODO: remove when core is free of json
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        ${Open3D_LIBRARIES}
        ${open3d_conversions_LIBRARIES}
        yaml-cpp
        )


# Main executable source
set(MAIN_SOURCE
        src/multiviperfrog_node.cpp
)

# Main executable
add_executable(${PROJECT_NAME}_node
        ${MAIN_SOURCE}
        )

# Link libraries to the main executable
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
