cmake_minimum_required(VERSION 2.8)
project( xytheta )
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})
add_executable( xytheta xytheta.cpp )
target_link_libraries(xytheta ${catkin_LIBRARIES} ${SBPL_LIBRARIES})

