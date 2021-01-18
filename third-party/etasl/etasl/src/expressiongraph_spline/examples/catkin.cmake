#IF(ENABLE_EXAMPLES)

 add_executable(spline1 examples/spline1.cpp )
 TARGET_LINK_LIBRARIES(spline1 ${PROJECT_NAME} ${catkin_LIBRARIES} )
 
 add_executable(spline2 examples/spline2.cpp )
 TARGET_LINK_LIBRARIES(spline2 ${PROJECT_NAME} ${catkin_LIBRARIES} )
 
 add_executable(spline3 examples/spline3.cpp )
 TARGET_LINK_LIBRARIES(spline3 ${PROJECT_NAME} ${catkin_LIBRARIES} )
 
 add_executable(spline4 examples/spline4.cpp )
 TARGET_LINK_LIBRARIES(spline4 ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
 
 add_executable(spline5 examples/spline5.cpp )
 TARGET_LINK_LIBRARIES(spline5 ${PROJECT_NAME} ${catkin_LIBRARIES} )

 add_executable(splines_tst examples/splines_tst.cpp )
 TARGET_LINK_LIBRARIES(splines_tst ${PROJECT_NAME} ${catkin_LIBRARIES} )

 add_executable(motionmodel examples/motionmodel_ex.cpp)
 TARGET_LINK_LIBRARIES(motionmodel ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#ENDIF(ENABLE_EXAMPLES)  

