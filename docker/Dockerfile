FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TERM=xterm-256color
ENV ROS_DOMAIN_ID=30

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu git curl cmake python3-pip build-essential \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav2-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-yaml-cpp-vendor \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-launch \
    ros-humble-launch-ros \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/ros2_projects/tdk_30th/robot_fsm_v2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "alias sb='source install/setup.bash'" >> ~/.bashrc && \
    echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash", "-ic"]