FROM osrf/ros:noetic-desktop-full

ADD docker/sources.list /etc/apt

RUN apt-get update \
&& apt-get install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev unzip python3-tk\
&& mkdir /sad

COPY ./thirdparty/ /sad/

WORKDIR /sad/

RUN ls -la \
&& rm -rf ./Pangolin \
&& unzip ./Pangolin.zip \
&& mkdir ./Pangolin/build \
&& cmake ./Pangolin -B ./Pangolin/build \
&& make -j4 -C ./Pangolin/build install \
# && cmake ./g2o -B ./g2o/build \
# && make -j4 -C ./g2o/build install \
&& rm -rf /var/lib/apt/lists/* \
&& rm -rf /sad
