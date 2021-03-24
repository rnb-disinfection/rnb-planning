#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

find_package(PkgConfig REQUIRED)

#catkin_add_env_hooks(  99.kdl-typekit SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(etasl_py_examples)

catkin_package(
    CATKIN_DEPENDS ur_description
)

find_package(catkin REQUIRED COMPONENTS xacro)

# Xacro files
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.urdf.xacro)

foreach(it ${xacro_files})
  # remove .xacro extension
  string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  set(output_filename ${CMAKE_MATCH_1})
  message(STATUS output "file name ${output_filename}")
  # create a rule to generate ${output_filename} from {it}
  xacro_add_xacro_file(${it} ${output_filename})
  list(APPEND out_files ${output_filename})
endforeach(it)

# add an abstract target to actually trigger the builds
add_custom_target(ur10_xacro_files ALL DEPENDS ${out_files})


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY notebooks/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/notebooks
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
  PATTERN ".svn" EXCLUDE)

