add_executable(run_imu_integration
        run_imu_integration.cc
        )

target_link_libraries(run_imu_integration
        glog gflags ${PROJECT_NAME}.common
        )

add_executable(run_eskf_gins run_eskf_gins.cc)
target_link_libraries(run_eskf_gins
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )

add_executable(process_gnss process_gnss.cc)
target_link_libraries(process_gnss
        glog gflags ${PROJECT_NAME}.common ${PROJECT_NAME}.ch3
        )

add_library(${PROJECT_NAME}.ch3
        static_imu_init.cc
        utm_convert.cc
        # ieskf/nav_state_manifold.cc
        # ieskf/ieskf.cc

        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/utm.cc
        ${PROJECT_SOURCE_DIR}/thirdparty/utm_convert/tranmerc.cc
        )

target_link_libraries(${PROJECT_NAME}.ch3
        glog gflags ${PROJECT_NAME}.common
        )

