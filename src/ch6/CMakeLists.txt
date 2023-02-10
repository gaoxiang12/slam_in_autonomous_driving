add_library(${PROJECT_NAME}.ch6
        icp_2d.cc
        lidar_2d_utils.cc
        likelihood_field.cc
        occupancy_map.cc
        submap.cc
        mapping_2d.cc
        multi_resolution_likelihood_field.cc
        loop_closing.cc
        frame.cc
        )

target_link_libraries(${PROJECT_NAME}.ch6
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}.common
        )

add_executable(test_2dlidar_io
        test_2dlidar_io.cc
        )

target_link_libraries(test_2dlidar_io
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${third_party_libs}
        )

add_executable(test_2d_icp_s2s
        test_2d_icp_s2s.cc
        )

target_link_libraries(test_2d_icp_s2s
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        )

add_executable(test_2d_icp_likelihood
        test_2d_icp_likelihood.cc
        )

target_link_libraries(test_2d_icp_likelihood
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_occupancy_grid
        test_occupancy_grid.cc
        )

target_link_libraries(test_occupancy_grid
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_2d_mapping
        test_2d_mapping.cc
        )

target_link_libraries(test_2d_mapping
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(test_mr_matching
        test_mr_matching.cc
        )

target_link_libraries(test_mr_matching
        ${PROJECT_NAME}.ch6
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )
