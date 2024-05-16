cmake_minimum_required(VERSION 3.10)
project(mw_api)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# detect ROS version
if ($ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
    message(STATUS "ROS_VERSION: ${ROS_VERSION}")
    message(STATUS "ROS_DISTRO: $ENV{ROS_DISTRO}")
    message(STATUS "ROS_PYTHON_VERSION: $ENV{ROS_PYTHON_VERSION}")
endif ()

# set middleware
if (${ROS_VERSION} EQUAL 1)
    set(MW ros CACHE STRING "set MW")
endif ()
if (${ROS_VERSION} EQUAL 2)
    set(MW ros2 CACHE STRING "set MW")
endif ()
message(STATUS "MW: ${MW}")
if (NOT DEFINED MW OR NOT MW)
    message(FATAL_ERROR "MW is not set.")
endif ()
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${MW})
    message(FATAL_ERROR "MW is not found: ${CMAKE_CURRENT_SOURCE_DIR}/${MW}")
endif ()
add_definitions(-Dmw_${MW})

find_package(Boost REQUIRED COMPONENTS filesystem)
if(Boost_FOUND)
    message(STATUS "Boost found!")
    message(STATUS "Found Boost version: ${Boost_VERSION}")
    message(STATUS "Found Boost include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Found Boost libraries: ${Boost_LIBRARIES}")
else()
    message(FATAL_ERROR "Boost not found!")
endif()

if (NOT ${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    set(mw_code_name $ENV{ROS_DISTRO} PARENT_SCOPE)
endif ()

# basic settings for mw_api
add_library(mw_api INTERFACE)

target_compile_definitions(mw_api INTERFACE mw_${MW})

target_include_directories(mw_api INTERFACE
        ${Boost_INCLUDE_DIRS}
        ${MW}/include
)

target_link_libraries(mw_api INTERFACE
        ${Boost_LIBRARIES}
)

function(mw_add_dependency_deprecated IN_MW TARGET SCOPE)
    # I cherish my thought process, and I don't want to delete it directly.
    message(DEPRECATION "WARNING: mw_add_dependency_deprecated() is deprecated.")
    if (NOT ${IN_MW} STREQUAL ${MW})
        message(STATUS "Skip add dependency of ${IN_MW} because it is not for ${MW}")
        return()
    endif ()
    set(include_dirs)
    set(libs)
    foreach(depend IN LISTS ARGN)
        message(STATUS "${MW}_DEPENDS: " ${depend})
        find_package(${depend} REQUIRED)
        list(APPEND include_dirs ${${depend}_INCLUDE_DIRS})
        list(APPEND libs ${${depend}_LIBRARIES})
    endforeach()
    target_include_directories(${TARGET} ${SCOPE} ${include_dirs})
    target_link_libraries(${TARGET} ${SCOPE} ${libs})
endfunction()

function(mw_add_dependency IN_MW TARGET SCOPE)
    if (NOT ${IN_MW} STREQUAL ${MW})
        message(STATUS "Skip add dependency of ${IN_MW} because it is not for ${MW}")
        return()
    endif ()
    if (${MW} STREQUAL ros)
        find_package(catkin REQUIRED COMPONENTS ${ARGN})
        target_include_directories(${TARGET} ${SCOPE} ${catkin_INCLUDE_DIRS})
        target_link_libraries(${TARGET} ${SCOPE} ${catkin_LIBRARIES})
    endif ()
    if (${MW} STREQUAL ros2)
        find_package(ament_cmake REQUIRED)
        foreach(depend IN LISTS ARGN)
            find_package(${depend} REQUIRED)
        endforeach ()
        ament_target_dependencies(${TARGET} ${SCOPE} ${ARGN})
    endif ()
endfunction()

mw_add_dependency(ros mw_api INTERFACE roscpp std_msgs rosbag)
mw_add_dependency(ros2 mw_api INTERFACE rclcpp std_msgs rosbag2_cpp)

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    add_executable(mw_api_test mw_api_test.cpp)
    target_link_libraries(mw_api_test mw_api)
endif ()
