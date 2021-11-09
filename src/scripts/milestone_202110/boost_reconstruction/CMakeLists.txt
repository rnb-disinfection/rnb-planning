cmake_minimum_required(VERSION 3.5)
project(reconstruction_interface_py)

find_package(PythonInterp 2.7)
find_package(PythonLibs 2.7)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CUDA_STANDARD 17)

find_package(Open3D REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread python)

#MESSAGE("boost libs - ${Boost_LIBRARIES}")
#include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

#include_directories(/usr/include/eigen3)
#include_directories(/usr/local/include/open3d/3rdparty)
#include_directories(/usr/include/jsoncpp)
#include_directories(/home/jhkim/Open3D/build_cuda/mkl_include/src/ext_mkl_include/include)
#include_directories(/home/jhkim/Open3D/build_cuda/mkl_install/include)

#configure_file("/home/jhkim/Open3D/cpp/open3d/Open3D.h.in"
#               "/home/jhkim/Open3D/cpp/open3d/Open3D.h")
#configure_file("/home/jhkim/Open3D/cpp/open3d/Open3DConfig.h.in"
#               "/home/jhkim/Open3D/cpp/open3d/Open3DConfig.h")

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name
############################################################################
##############################  BUILD LIBRARY ##############################

add_library(${PROJECT_NAME} SHARED
	reconstruction_interface/reconstruction_interface.cpp
	reconstruction_interface/boost_python_interface.cpp
)

#target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} yaml-cpp)
target_link_libraries(${PROJECT_NAME} Open3D::Open3D ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
		#/usr/lib/x86_64-linux-gnu/libboost_python-py27.so.1.65.1
		#/usr/lib/x86_64-linux-gnu/libboost_python-py27.so
		#/usr/lib/x86_64-linux-gnu/libboost_python.so
		)


############################################################################
#############################  TEST EXECUTION ##############################
add_executable(${PROJECT_NAME}-test
        reconstruction_interface/test_main.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} Open3D::Open3D ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
#############################  TEST EXECUTION ##############################
############################################################################
