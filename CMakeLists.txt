
cmake_minimum_required(VERSION 3.10)
project(stanley_control)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)




add_subdirectory(autonomous_proto)
add_subdirectory(mw_api)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
if(GeographicLib_FOUND)
        message(STATUS "GeographicLib found!")
else()
        message(FATAL_ERROR "GeographicLib not found!")
endif()



add_executable(stanley_control
        src/main.cpp
        src/stanley.cpp
        src/control.cpp)

target_include_directories(stanley_control PUBLIC
        ${GeographicLib_INCLUDE_DIRS}
        include
)



target_link_libraries(stanley_control PUBLIC
        autonomous_proto
        mw_api
        ${GeographicLib_LIBRARIES}
        yaml-cpp
        dl
        )