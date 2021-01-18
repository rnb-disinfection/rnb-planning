# If the pyx file is a C++ file, we should specify that here.
# set( CYTHON_CXX_EXTENSION "cpp" )

set_source_files_properties(
   ${CMAKE_CURRENT_SOURCE_DIR}/etasl.pyx
   PROPERTIES CYTHON_IS_CXX TRUE )

# CAUTION:
#  - You can specify only one .pyx file for each cython module.
#  - The name of the module and the .pyx file should be identical.
cython_add_module(etasl etasl.pyx etaslcppdriver.cpp luactx.cpp)

# The library (libros_cython_example.so) is created in upper CMakeLists.txt
target_link_libraries(etasl etasl_py ${catkin_LIBRARIES})

# Set the targets to be copied in the python package directry.
# This example only place mul_two_ints and fibonacci modules.
# we can call C version of these and python version of the rest.
set_target_properties(etasl
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(TARGETS etasl 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

