add_executable(motion motion.cc)
target_link_libraries(motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
