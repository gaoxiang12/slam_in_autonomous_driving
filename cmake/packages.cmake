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
