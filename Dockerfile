FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle
ENV ROS_DOMAIN_ID=0

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-simulations \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle" >> /root/.bashrc

# Default: interactive shell (run launch & dispatcher commands manually)
CMD ["bash"]
