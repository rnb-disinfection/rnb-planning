cmake_minimum_required(VERSION 2.8.3)
project(luabind)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)
include_directories( . ${catkin_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(LUA REQUIRED lua5.1)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${LUA_INCLUDE_DIRS})
LINK_DIRECTORIES(${LUA_LIBRARY_DIRS})


# BUILDING AND LINKING LIBRARY
FILE( GLOB LUABIND_SRCS src/[^.]*.cpp src/[^.]*.cxx)

add_library(${PROJECT_NAME} ${LUABIND_SRCS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LUA_LIBRARIES})


catkin_package(
  INCLUDE_DIRS . 
  LIBRARIES ${PROJECT_NAME}
  DEPENDS LUA
)

# BUILDING TEST
FILE( GLOB TST_SRCS test/[^.]*.cpp)

message(STATUS alltestfiles "  " ${TST_SRCS})

FOREACH( TST_SRC ${TST_SRCS})
    message(STATUS generating ${TST_SRC})
    get_filename_component( TGT ${TST_SRC} NAME_WE)
    message(STATUS target ${TGT})
    add_executable( ${TGT}  test/main.cxx ${TST_SRC})
    target_link_libraries(${TGT} ${PROJECT_NAME} ${LUA_LIBRARIES})
ENDFOREACH ( TST_SRC)

target_link_libraries(${TGT} ${PROJECT_NAME} ${LUA_LIBRARIES})



# POTENTIALLY, BUILDING AND LINKING EXAMPLES
#OPTION(ENABLE_EXAMPLES "enable compilation of a series of examples" ON) 
#INCLUDE(${PROJECT_SOURCE_DIR}/examples/CMakeLists.txt)




# INSTALLING LIBRARY AND HEADER-FILES
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY luabind/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

