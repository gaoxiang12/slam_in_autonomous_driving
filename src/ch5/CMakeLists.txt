add_executable(point_cloud_load_and_vis point_cloud_load_and_vis.cc)
target_link_libraries(point_cloud_load_and_vis
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        gflags
        )

add_executable(pcd_to_bird_eye pcd_to_bird_eye.cc)
target_link_libraries(pcd_to_bird_eye
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        )

add_executable(scan_to_range_image scan_to_range_image.cc)
target_link_libraries(scan_to_range_image
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        )

add_library(${PROJECT_NAME}.ch5
        bfnn.cc
        kdtree.cc
        octo_tree.cc
        )
target_link_libraries(${PROJECT_NAME}.ch5
        tbb
        )

add_executable(linear_fitting linear_fitting.cc)
target_link_libraries(linear_fitting
        ${PCL_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        gflags
        )

ADD_EXECUTABLE(test_nn test_nn.cc)
ADD_TEST(test_nn test_bfnn)

target_link_libraries(test_nn
        gtest pthread glog gflags ${PROJECT_NAME}.ch5 ${PROJECT_NAME}.common ${PCL_LIBRARIES} tbb
        )
