add_library(${PROJECT_NAME}.ch9
        frontend.cc
        keyframe.cc
        optimization.cc
        loopclosure.cc
        )

target_link_libraries(${PROJECT_NAME}.ch9
        ${PROJECT_NAME}.ch8
        ${PROJECT_NAME}.ch4
        ${PROJECT_NAME}.ch3
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(run_frontend run_frontend.cc)
target_link_libraries(run_frontend
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )

add_executable(run_optimization run_optimization.cc)
target_link_libraries(run_optimization
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )

add_executable(run_loopclosure run_loopclosure.cc)
target_link_libraries(run_loopclosure
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )

add_executable(run_mapping run_mapping.cc)
target_link_libraries(run_mapping
        ${PROJECT_NAME}.ch9
        ${third_party_libs}
        )

add_executable(dump_map dump_map.cc)
target_link_libraries(dump_map
        ${PROJECT_NAME}.ch9
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(split_map split_map.cc)
target_link_libraries(split_map
        ${PROJECT_NAME}.ch9
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )
