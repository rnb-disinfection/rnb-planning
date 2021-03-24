#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

add_executable(two_cylinders two_cylinders.cpp)
target_link_libraries(two_cylinders ${PROJECT_NAME})


add_executable(two_cylindersv2 two_cylindersv2.cpp)
target_link_libraries(two_cylindersv2 ${PROJECT_NAME})

add_executable(two_cylinders_with_input two_cylinders_with_input.cpp)
target_link_libraries(two_cylinders_with_input ${PROJECT_NAME})

add_executable(kuka_ex kuka_ex.cpp)
target_link_libraries(kuka_ex ${PROJECT_NAME})

add_executable(test_urdf test_urdf.cpp)
target_link_libraries(test_urdf ${PROJECT_NAME})




