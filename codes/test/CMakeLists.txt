cmake_minimum_required(VERSION 2.8)
project( test )
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})
add_executable( test test.cpp )


##Add these lines to use C++11
set_property(TARGET test PROPERTY CXX_STANDARD 11)
set_property(TARGET test PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(test ${catkin_LIBRARIES} ${SBPL_LIBRARIES})

