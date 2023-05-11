add_library(${PROJECT_NAME}.ch7
        icp_3d.cc
        ndt_3d.cc
        gen_simu_data.cc
        direct_ndt_lo.cc
        incremental_ndt_lo.cc
        ndt_inc.cc
        loam-like/feature_extraction.cc
        loam-like/loam_like_odom.cc

        loosely_coupled_lio/loosely_lio.cc
        loosely_coupled_lio/cloud_convert.cc
        loosely_coupled_lio/measure_sync.cc
        )

target_link_libraries(${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_icp
        test/test_icp.cc
        )

target_link_libraries(test_icp
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_ndt_lo
        test/test_ndt_lo.cc
        )

target_link_libraries(test_ndt_lo
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_inc_ndt_lo
        test/test_inc_ndt_lo.cc
        )

target_link_libraries(test_inc_ndt_lo
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_feature_extraction
        test/test_feature_extraction.cc
        )

target_link_libraries(test_feature_extraction
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_loam_odom
        test/test_loam_odom.cc
        )

target_link_libraries(test_loam_odom
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_loosely_lio
        test/test_loosely_lio.cc
        )

target_link_libraries(test_loosely_lio
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(run_gen_simu_data
        test/run_gen_simu_data.cc
        )

target_link_libraries(run_gen_simu_data
        ${PROJECT_NAME}.ch5
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )
