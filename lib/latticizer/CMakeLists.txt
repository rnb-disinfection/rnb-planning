cmake_minimum_required(VERSION 3.5)
project(latticizer)

set(CMAKE_CXX_STANDARD 14)

find_package (Python2 COMPONENTS Interpreter Development)
set(BOOST_PYTHONLIB python2)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread python)


include_directories(include)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Python2_INCLUDE_DIRS})

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

############################################################################
##############################  BUILD LIBRARY ##############################
add_library(${PROJECT_NAME} SHARED
        src/latticizer.cpp
        src/boost_python_interface.cpp)
target_link_libraries(${PROJECT_NAME} ${Python2_LIBRARIES} ${Boost_LIBRARIES})
##############################  BUILD LIBRARY ##############################
############################################################################


############################################################################
#############################  TEST EXECUTION ##############################
add_executable(${PROJECT_NAME}-test src/test.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${Boost_LIBRARIES})
#############################  TEST EXECUTION ##############################
############################################################################
