add_library(${PROJECT_NAME}.ch8
        lio-iekf/lio_iekf.cc
        lio-preinteg/lio_preinteg.cc
        )

target_link_libraries(${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(test_lio_iekf test/test_lio_iekf.cc)
target_link_libraries(test_lio_iekf
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch4
        )

add_executable(test_lio_preinteg test/test_lio_preinteg.cc)
target_link_libraries(test_lio_preinteg
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch7
        ${PROJECT_NAME}.ch4
        )
