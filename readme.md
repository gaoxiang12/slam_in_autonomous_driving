## SLAM in Autonomous Driving book (SAD book)

本书向读者系统介绍了惯性导航、组合导航、激光建图、激光定位、激光惯导里程计等知识。本仓库是书籍对应的源代码仓库，可以公开使用。

## 注意

- 本书文本目前处于审阅状态。如果您希望成为本书的审稿人，请联系：gao.xiang.thu at gmail.com
- 成为审稿人之后，您可以查阅本书每日更新的PDF稿件。同时，您需要在两个月内向我反馈您的意见。您可以通过github
  issue或邮件形式将意见发送给我。
- 如果您有意向为本书写几句推荐语，也请联系我本人。推荐语会出现在本书前言部分或者书的封底部分。

## 本书的内容编排

- 第1章，概述
- 第2章，数学基础知识回顾，几何学、运动学、KF滤波器理论，矩阵李群
- 第3章，误差状态卡尔曼滤波器，惯性导航、卫星导航、组合导航
- 第4章，预积分，图优化，基于预积分的组合导航
- 第5章，点云基础处理，各种最近邻结构，点云线性拟合
- 第6章，2D激光建图，scan matching, 似然场，子地图，2D回环检测，pose graph
- 第7章，3D激光建图，ICP，变种ICP，NDT，NDT LO, Loam-like LO，LIO松耦合
- 第8章，紧耦合LIO，IESKF，预积分紧耦合LIO
- 第9章，离线建图，前端，后端，批量回环检测，地图优化，切片导出
- 第10章，融合定位，激光定位，初始化搜索，切片地图加载，EKF融合

## 本书的特点

- 在这本书里，您会复现许多激光SLAM中的经典算法和数据结构。
    - 您需要自己推导、实现一个误差状态卡尔曼滤波器(ESKF)，把IMU和GNSS的数据喂给它，看它如何推算自己的状态。
    - 您还会用预积分系统实现一样的功能，然后对比它们的运行方式。
    - 接下来您会实现一遍2D激光SLAM中的常见算法：扫描匹配、似然场、子地图，占据栅格，再用回环检测来构建一个更大的地图。这些都需要您自己来完成。
    - 在激光SLAM中，您也会自己实现一遍Kd树，处理近似最近邻，然后用这个Kd树来实现ICP，点面ICP，讨论它们有什么可以改进的地方。
    - 然后您会实现经典的NDT算法，测试它的配准性能，然后用它来搭建一个激光里程计。它比大部分现有LO快得多。
    - 您也会实现一个点面ICP的激光里程计，它也非常快。它工作的方式类似于Loam，但更简单。
    - 您会想要把IMU系统也放到激光里程计中。我们会实现松耦合和紧耦合的LIO系统。同样地，您需要推导一遍迭代卡尔曼滤波器和预积分图优化。
    - 您需要把上面的系统改成离线运行的，让回环检测运行地充分一些。最后将它做成一个离线的建图系统。
    - 最后，您可以对上述地图进行切分，然后用来做实时定位。
- 本书的大部分实现都要比类似的算法库简单的多。您可以快速地理解它们的工作方式，不需要面对复杂的接口。
- 本书会使用非常方便的并发编程。您会发现，本书的实现往往比现有算法更高效。当然这有一部分是历史原因造成的。
- 本书每章都会配有动态演示，像这样：

![](./doc/lio_demo.gif)
![](./doc/2dmapping_demo.gif)
![](./doc/lo_demo.gif)

希望您能喜欢本书的极简风格，发现算法的乐趣所在。

## 数据集

- 数据集下载链接：
- 百度云链接: https://pan.baidu.com/s/1ELOcF1UTKdfiKBAaXnE8sQ?pwd=feky 提取码: feky

- 包含以下数据集。总量较大(270GB)，请视自己硬盘容量下载。
    - UrbanLoco (ULHK，3D激光，道路场景)
    - NCLT (3D激光，RTK，校园场景)
    - WXB (3D激光，园区场景)
    - 2dmapping (2D激光，商场场景)
    - AVIA (大疆固态激光)
    - UTBM (3D激光，道路场景)
- 其他的内置数据
    - 第3,4章使用文本格式的IMU，RTK数据
    - 第7章使用了一部分EPFL的数据作为配准点云来源
- 您应该将上述数据下载至./dataset/sad/目录下，这样许多默认参数可以正常工作。如果不那么做，您也可以手动指定这些文件路径。如果您硬盘容量不足，可以将其他硬盘的目录软链至此处。

## 编译

- 本书推荐的编译环境是Ubuntu 20.04。更老的Ubuntu版本需要适配gcc编译器，主要是C++17标准。更新的Ubuntu则需要您自己安装对应的ROS版本。
- 在编译本书代码之前，请编译安装本书thirdparty/下的```g2o```以及以下三方库（如果您机器上没有安装的话）
   - ROS Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
   - pcl-ros (如果没有选择安装 ```ros-noetic-desktop-full```): ```sudo apt install ros-noetic-pcl-ros```
   - velodyne-msgs (如果没有选择安装```ros-noetic-desktop-full```): ```sudo apt install ros-noetic-velodyne-msgs```
   - opencv: ```sudo apt install libopencv-dev```
   - glog: ```sudo apt install libgoogle-glog-dev```
   - eigen3: ```sudo apt install libeigen3-dev```
   - suitesparse: ```sudo apt install libsuitesparse-dev```
   - pcl: ```sudo apt install libpcl-dev```
   - yaml-cpp: ```sudo apt install libyaml-cpp-dev```
   - tbb: ```sudo apt install libbtbb-dev```
   - gmock: ```sudo apt install libgmock-dev```
   - 以下命令可以一次安装上面几个库: ```sudo apt install -y libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev```
   - Pangolin: https://github.com/stevenlovegrove/Pangolin
- 之后，使用通常的cmake, make 方式就可以编译本书所有内容了。

## TODO项

- 将UI里的不必要信息去除
- 整理数据集
- 去除不必要的msg定义(尽量使用ros标准消息)

## NOTES

- [已确认] ULHK的IMU似乎和别家的不一样，已经去了gravity
- [已确认] NCLT的IMU在转包的时候转成了Lidar系，于是Lidar与IMU之间没有旋转的外参（本来Lidar是转了90度的），现在Lidar是X左Y后Z下，原车是X前Y右Z下。本书使用的NCLT数据均基于点云系,
  IMU的杆臂被忽略。
- [已确认] NCLT的rtk fix并不是非常稳定，平均误差在米级
