cmake_minimum_required(VERSION 3.0)

# set project name from current directory
get_filename_component(BASENAME ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${BASENAME})

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "-g -Wall")
set(CMAKE_CXX_FLAGS "-g -Wall")

find_package (OpenCV REQUIRED NO_CMAKE_FIND_ROOT_PATH)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()


# define executable to build
include_directories("libv4l2")
include_directories("ArmorDetector")
include_directories("General")
aux_source_directory(libv4l2 SRC_FILES)
aux_source_directory(ArmorDetector SRC_FILES)
aux_source_directory(General SRC_FILES)
add_library(v4l2cpp SHARED ${SRC_FILES})
target_link_libraries(v4l2cpp ${OpenCV_LIBS})

add_executable(client test.cpp)
target_link_libraries(client v4l2cpp ${OpenCV_LIBS})
