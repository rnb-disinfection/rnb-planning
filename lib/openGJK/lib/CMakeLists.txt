# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
#                                    #####        # #    #                #
#        ####  #####  ###### #    # #     #       # #   #                 #
#       #    # #    # #      ##   # #             # #  #                  #
#       #    # #    # #####  # #  # #  ####       # ###                   #
#       #    # #####  #      #  # # #     # #     # #  #                  #
#       #    # #      #      #   ## #     # #     # #   #                 #
#        ####  #      ###### #    #  #####   #####  #    #                #
#                                                                         #
#   This file is part of openGJK.                                         #
#                                                                         #
#   openGJK is free software: you can redistribute it and/or modify       #
#    it under the terms of the GNU General Public License as published by #
#    the Free Software Foundation, either version 3 of the License, or    #
#    any later version.                                                   #
#                                                                         #
#    openGJK is distributed in the hope that it will be useful,           #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of       #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See The        #
#    GNU General Public License for more details.                         #
#                                                                         #
#   You should have received a copy of the GNU General Public License     #
#    along with Foobar.  If not, see <https://www.gnu.org/licenses/>.     #
#                                                                         #
#        openGJK: open-source Gilbert-Johnson-Keerthi algorithm           #
#             Copyright (C) Mattia Montanari 2018 - 2019                  #
#               http://iel.eng.ox.ac.uk/?page_id=504                      #
#                                                                         #
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

cmake_minimum_required(VERSION 3.5)
project (openGJKlib)

set(CMAKE_CXX_STANDARD 11)

# SELECT USER OPTIONS
option(VERSION_ACCURATE  "Reduce speed to maximise accuracy (OFF)"   OFF )

# APPLY USER OPTIONS
IF(VERSION_ACCURATE)
    set(USE_PREDICATES          ON)
    set(openGJK_VERSION "Accurate")
ELSE()
    set(USE_PREDICATES         OFF)
    set(openGJK_VERSION     "Fast")
ENDIF()

# COMPILE
message( "[${CMAKE_PROJECT_NAME}] Compiling ..")
message(STATUS "Version     (Accurate,Fast): " ${openGJK_VERSION}  )
message(STATUS "Build type  (Debug,Release): " ${CMAKE_BUILD_TYPE} )

# Select source files
set( SOURCE_FILES src/openGJK.cpp src/openGJK_cpp_wrap.cpp src/boost_python_interface.cpp)
set( SOURCE_HEADS include/openGJK/openGJK.h include/openGJK_cpp_wrap.h include/bp_container_interface.h)

IF(USE_PREDICATES)
    # for adpative floating-point artim.
    set( SOURCE_FILES ${SOURCE_FILES} ext/predicates.c)
    set( SOURCE_HEADS ${SOURCE_HEADS} ext/predicates.h)
    # Add flag for adpative floating-point artim.
    add_definitions(-DADAPTIVEFP)
ENDIF()

find_package(PythonLibs 2.7 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

# Create the (dynamic) library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${SOURCE_HEADS})
add_definitions(-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=TRUE -DBUILD_SHARED_LIBS=FALSE)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

IF(USE_PREDICATES)
    # for adpative floating-point artim.
    target_include_directories( ${PROJECT_NAME}
            PUBLIC ${PROJECT_SOURCE_DIR}/ext
            )
ENDIF()

# Report
message( ".. DONE!")
