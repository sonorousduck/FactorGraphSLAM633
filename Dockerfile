# Other destros may not be supported with all of the packages listed below
ARG ROS_DISTRO=humble 

# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:${ROS_DISTRO}-ros-core

# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV MAIN_WS /docker_SLAM
ENV ROS_WS ${MAIN_WS}/ros2_ws
ENV DRIVER_WS ${MAIN_WS}/drivers_ros2_ws

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    # python3-pcl \
    # python3-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# install nano and pip
RUN apt-get update && apt-get install -y nano \
    python3-pip \
    && pip3 install --upgrade pip

# Install Segmentation Libraries
RUN python3 -m pip install numpy matplotlib opencv-python open3d ouster-sdk 
# RUN python3 -m pip install torch==1.12.1+cu113 torchvision==0.13.1+cu113 --extra-index-url https://download.pytorch.org/whl/cu113 
# RUN python3 -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
# RUN git clone https://github.com/bytedance/kmax-deeplab.git && cd kmax-deeplab && python3 -m pip install -r requirements.txt

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install ros-${ROS_DISTRO}-gtsam cmake vim tmux -y && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 

# Install Livox SDK (Needed for Fast LIO-SAM)
# RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && cd ./Livox-SDK2/ && mkdir build && cd build && cmake .. && make -j && make install

# Install ros2 driver for Livox SDK 
# RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
#     . /opt/ros/${ROS_DISTRO}/setup.sh && \
#     cd ws_livox/src/livox_ros_driver2 && \
#     ./build.sh ${ROS_DISTRO} && \
#     echo "source /ws_livox/install/setup.bash" >> ~/.bashrc

# # Install Fast-LIO and Ouster package
# RUN mkdir -p /docker_yride/other_drivers_ros2_ws/src && \
#   cd /docker_yride/other_drivers_ros2_ws/src && \
#   git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
#   git clone https://github.com/BYU-FRoSt-Lab/FAST_LIO.git --recursive && \
#   cd FAST_LIO && \
#   git checkout ROS2 && git pull && \
#   cd /docker_yride/other_drivers_ros2_ws && \
#   . /ws_livox/install/setup.sh && \
#   rosdep install --from-paths src --ignore-src -y && \
#   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#   echo "source /docker_yride/other_drivers_ros2_ws/install/setup.bash" >> ~/.bashrc

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install LIO-SAM
RUN apt-get update && sudo apt install ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-xacro -y \
    ros-${ROS_DISTRO}-spatio-temporal-voxel-layer \
    ros-${ROS_DISTRO}-octomap-server 

RUN mkdir -p ${DRIVER_WS}/src && \
    cd ${DRIVER_WS}/src && \
    git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git && \
    # cd ouster-ros && git fetch --all --tags && git checkout tags/ros2-v0.13.1 && cd .. && \
    # git clone https://github.com/TixiaoShan/LIO-SAM.git && \
    # cd LIO-SAM && git checkout ros2 && git pull && \
    # . /ws_livox/install/setup.sh && \
    cd .. && \
    # rosdep install --from-paths src --ignore-src -y && 
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    # echo "source /docker_yride/other_drivers_ros2_ws/install/setup.bash" >> ~/.bashrc
# ADD ros2_ws/liosam_params.yaml params.yaml
# RUN mv params.yaml /docker_yride/other_drivers_ros2_ws/src/LIO-SAM/config/params.yaml


# The RAW param is thorwing an error. 
RUN cd ${DRIVER_WS}/src/ouster-ros/ouster-ros/config/ && \
  sed -i 's/proc_mask: IMU|PCL|SCAN|IMG|RAW/proc_mask: IMU|PCL|SCAN|IMG/' driver_params.yaml && \
  sed -i 's/use_system_default_qos: false/use_system_default_qos: true/' driver_params.yaml


# Install ros2 IMU + LUCID Camera + Velodyne
RUN apt-get update && apt-get install ros-${ROS_DISTRO}-microstrain-inertial-driver ros-${ROS_DISTRO}-microstrain-inertial-rqt -y

# RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#   && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
#     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && apt-get update && apt-get install -y nvidia-container-toolkit

###########################################################################################################
#                                        Do this Last                                                     #
###########################################################################################################
# Changes ownership of the container to the host user so that files created in the
# container are owned by the host user. This is necessary for the host user to be able to
# delete or modify files created in the container after the container has stopped.
# Define build arguments for UID and GID (Set in docker_compose.yml)
# # Arguments from docker_compose.yml
# ARG UID 
# ARG GID
# ARG UNAME
# ARG PASSWORD  # Add an argument for the user's password

# Install sudo and other necessary tools
RUN apt-get update && apt-get install -y sudo

# Create group and user with provided UID and GID
# RUN groupadd -g $GID $UNAME && \
    # useradd -m -u $UID -g $GID -s /bin/bash $UNAME && \
    # usermod -aG sudo $UNAME && \
    # echo "$UNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
    # echo "$UNAME:$PASSWORD" | chpasswd

# Set up the workspace and switch to the new user
# USER $UNAME
WORKDIR ${ROS_WS}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${DRIVER_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

    # echo "source /ws_livox/install/setup.bash" >> ~/.bashrc
