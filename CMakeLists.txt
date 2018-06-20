cmake_minimum_required(VERSION 2.6)

project(DouplePrecision)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Enable C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-enum-compare -Wno-deprecated-declarations")

# Find the libraries
find_package(EIGEN REQUIRED)

set(source_files src/main.cpp)

#Déclaration de l'exécutable

set(EXEC_NAME doubleprecision)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/")

include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(${EXEC_NAME} ${source_files})
