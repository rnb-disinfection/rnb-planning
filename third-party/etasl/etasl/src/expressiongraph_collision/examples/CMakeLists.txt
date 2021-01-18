IF(ENABLE_EXAMPLES)

  set(EXAMPLES
      #fcl_tst1
      #fcl_tst3
    extract_capsule
  )

 #add_executable(fcl_tst1 examples/fcl_tst1.cpp )
 #TARGET_LINK_LIBRARIES(fcl_tst1 ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

 #add_executable(fcl_tst3 examples/fcl_tst3.cpp )
 #TARGET_LINK_LIBRARIES(fcl_tst3 ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

 add_executable(extract_capsule examples/extract_capsule.cpp )
 TARGET_LINK_LIBRARIES(extract_capsule ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
 
 # add_executable(test_ccd examples/test_ccd.c )
 #TARGET_LINK_LIBRARIES(test_ccd ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ccd)



 #add_executable(test_fcl_capsule_box_1 examples/test_fcl_capsule_box_1.cpp)
 #TARGET_LINK_LIBRARIES(test_fcl_capsule_box_1 ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
ENDIF(ENABLE_EXAMPLES)  

