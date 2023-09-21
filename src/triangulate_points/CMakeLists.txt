add_executable(triangulate_points triangulate_points.cc)
target_link_libraries(triangulate_points
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )

add_executable(odo odo.cc)
target_link_libraries(odo
${PROJECT_NAME}.common
${PROJECT_NAME}.tools
)


add_executable(fisheye_pix_2_cam_test fisheye_pix_2_cam_test.cc)
target_link_libraries(fisheye_pix_2_cam_test
${PROJECT_NAME}.common
${PROJECT_NAME}.tools
)