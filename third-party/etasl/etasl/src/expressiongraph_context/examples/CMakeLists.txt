#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
if(NOT USE_ROSBUILD)
    
    add_executable(test_itasc test_itasc.cpp)
    target_link_libraries(test_itasc ${PROJECT_NAME})

    add_executable(test_inputchannels test_inputchannels.cpp)
    target_link_libraries(test_inputchannels ${PROJECT_NAME})

    add_executable(test_outputchannels test_outputchannels.cpp)
    target_link_libraries(test_outputchannels ${PROJECT_NAME})

    add_executable(test_solver test_solver.cpp)
    target_link_libraries(test_solver ${PROJECT_NAME})

elseif()
    
    rosbuild_add_executable(test_itasc test_itasc.cpp)
    target_link_libraries(test_itasc ${PROJECT_NAME})

    rosbuild_add_executable(test_inputchannels test_inputchannels.cpp)
    target_link_libraries(test_inputchannels ${PROJECT_NAME})

    rosbuild_add_executable(test_outputchannels test_outputchannels.cpp)
    target_link_libraries(test_outputchannels ${PROJECT_NAME})

    rosbuild_add_executable(test_solver test_solver.cpp)
    target_link_libraries(test_solver ${PROJECT_NAME})

endif()








