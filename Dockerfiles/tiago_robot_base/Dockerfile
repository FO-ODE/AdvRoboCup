# modified from Tiago official Dockerfile
# adjust the moveit version for TIAGo Robot in reality

# 基础镜像
FROM osrf/ros:noetic-desktop-full-focal

LABEL maintainer="Yue Erro <yue.erro@pal-robotics.com>"

ARG REPO_WS=/tiago_public_ws
RUN mkdir -p $REPO_WS/src
WORKDIR $REPO_WS

# 安装依赖工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    libv4l-dev \
    libv4l2rds0 \
    git \
    wget \
    vim \
    locales \
    dpkg \
    ssh \
    curl \
    aptitude \
    g++ \
    gcc \
    openvpn \
    gnupg \
    bash-completion \
    vim-gtk3 \
    nano \
    psmisc \
    ccache \
    gdb \
    qtcreator \
    htop \
    man \
    meld \
    silversearcher-ag \
    terminator \
    tig \
    valgrind \
    iputils-ping \
    ipython3 \
    python-is-python3 \
    python3-scipy \
    python3-wstool \
    python3-networkx \
    python3-pip  \
    python3-vcstool \
    python3-rosinstall \
    python3-catkin-tools \
    ros-noetic-actionlib-tools \
    && rm -rf /var/lib/apt/lists/*

# 下载 TIAGo 工程
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall \
    && vcs import src < tiago_public-noetic.rosinstall

# 拷贝 MoveIt 的 DEB 包
COPY tiago_moveit /opt/tiago_moveit
WORKDIR /opt/tiago_moveit

# 先卸载冲突的 MoveIt 相关包，再手动安装所需 deb
RUN apt-get update && \
    apt-get remove -y ros-noetic-moveit-msgs ros-noetic-moveit-core ros-noetic-moveit-kinematics || true && \
    dpkg -i *.deb || apt-get install -f -y && \
    rm -rf /var/lib/apt/lists/*

# 回到主工作目录
WORKDIR $REPO_WS

# 安装 ROS 依赖
ARG ROSDEP_IGNORE="urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

RUN apt-get update && rosdep install --from-paths src --ignore-src -y --rosdistro noetic --skip-keys="${ROSDEP_IGNORE}"

# 构建
RUN bash -c "source /opt/ros/noetic/setup.bash && \
    catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr \`nproc\` / 2) && \
    echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc"

ENTRYPOINT ["bash"]
