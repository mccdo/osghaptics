cmake_minimum_required(VERSION 2.6.3)

set(CMAKE_BUILD_TYPE Release)

set(ProjectName osghaptics)
list( APPEND CMAKE_MODULE_PATH "${osghaptics_SOURCE_DIR}/CMakeModules" )

project(${ProjectName})

find_package( OpenGL )

set(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
make_directory(${OUTPUT_LIBDIR})
set(LIBRARY_OUTPUT_PATH ${OUTPUT_LIBDIR})


add_subdirectory(src)



