# 引入该目录下的.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# livox ros driver
add_subdirectory(thirdparty/livox_ros_driver)

include_directories(${CMAKE_BINARY_DIR}/devel/include) # 引用ros生成的msg header

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o 使用thirdparty中的
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/)
set(g2o_libs
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
	# ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_cholmod.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_dense.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARY}
        )

# ros
# 为了2D scan, pointcloud2
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(${catkin_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

# 其他thirdparty下的内容
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/velodyne/include)

if(BUILD_WITH_UBUNTU1804)
    function(extract_file filename extract_dir)
        message(STATUS "Extract ${filename} to ${extract_dir} ...")
        set(temp_dir ${extract_dir})
        if(EXISTS ${temp_dir})
            file(REMOVE_RECURSE ${temp_dir})
        endif()
        file(MAKE_DIRECTORY ${temp_dir})
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xvzf ${filename}
                WORKING_DIRECTORY ${temp_dir})
    endfunction()

    set(TBB_ROOT_DIR ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8)
    set(TBB_BUILD_DIR "tbb_build_dir=${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
    set(TBB_BUILD_PREFIX "tbb_build_prefix=tbb")

    extract_file(${PROJECT_SOURCE_DIR}/thirdparty/tbb/2019_U8.tar.gz ${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8)

    include(${TBB_ROOT_DIR}/cmake/TBBBuild.cmake)

    #message(STATUS "======TBB_BUILD_DIR = ${TBB_BUILD_DIR}")
    #message(STATUS "======TBB_BUILD_PREFIX = ${TBB_BUILD_PREFIX}")

    tbb_build(TBB_ROOT ${TBB_ROOT_DIR}
            compiler=gcc-9
            stdver=c++17
            ${TBB_BUILD_DIR}
            ${TBB_BUILD_PREFIX}
            CONFIG_DIR
            TBB_DIR)

    find_package(TBB REQUIRED)

    include_directories(${PROJECT_SOURCE_DIR}/thirdparty/tbb/oneTBB-2019_U8/oneTBB-2019_U8/include)
    link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/tbb_release)

    set(third_party_libs
            ${catkin_LIBRARIES}
            ${g2o_libs}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${Pangolin_LIBRARIES}
            glog gflags
            ${yaml-cpp_LIBRARIES}
            yaml-cpp
            TBB::tbb
            )
else()
    set(third_party_libs
            ${catkin_LIBRARIES}
            ${g2o_libs}
            ${OpenCV_LIBS}
            ${PCL_LIBRARIES}
            ${Pangolin_LIBRARIES}
            glog gflags
            ${yaml-cpp_LIBRARIES}
            yaml-cpp
            tbb
            )
endif ()