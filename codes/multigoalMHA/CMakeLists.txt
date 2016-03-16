cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project( multigoalMHA )
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

add_executable( multigoalMHA multigoalMHA.cpp  )

<<<<<<< HEAD
##Add these lines to use C++11
=======
>>>>>>> 8d8c489adac737747350b5320d357075c049ef40
set_property(TARGET multigoalMHA PROPERTY CXX_STANDARD 11)
set_property(TARGET multigoalMHA PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(multigoalMHA ${catkin_LIBRARIES} ${SBPL_LIBRARIES} sbpl)

