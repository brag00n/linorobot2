ARG ROS_DISTRO=

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as base
SHELL ["/bin/bash", "-c"]

ARG BASE_GIT_LINOROBOT2

#rviz fix https://github.com/ros2/rviz/issues/948
RUN apt-get update -q \
    && apt-get install -y software-properties-common \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get install libegl-mesa0 libegl1-mesa-dev \
    libgbm-dev libgbm1 libgl1-mesa-dev \
    libgl1-mesa-dri libglapi-mesa libglx-mesa0 wget -y 

RUN sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO    

RUN mkdir -p /root/linorobot2_ws/src
WORKDIR /root/linorobot2_ws

RUN apt-get install -y git-all
RUN git clone -b docker https://github.com/${BASE_GIT_LINOROBOT2}/linorobot2.git  src/linorobot2
RUN touch src/linorobot2/linorobot2_gazebo/COLCON_IGNORE
RUN apt-get update && rosdep install --from-path src --ignore-src -y --skip-keys micro_ros_agent
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

#https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
WORKDIR /root
RUN wget -O .shm_off.xml https://raw.githubusercontent.com/eProsima/Fast-DDS/107ea8d64942102696840cd7d3e4cf93fa7a143e/examples/cpp/dds/AdvancedConfigurationExample/shm_off.xml

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /root/linorobot2_ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /
RUN sed -i "s/rosdistro/$ROS_DISTRO/" /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

FROM base AS simulation

RUN sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO    

WORKDIR /root/linorobot2_ws
RUN rm src/linorobot2/linorobot2_gazebo/COLCON_IGNORE
RUN rosdep install --from-path src --ignore-src -y --skip-keys micro_ros_agent
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    colcon build --packages-select linorobot2_gazebo

WORKDIR /root/linorobot2_ws
RUN rm -rf src/linorobot2/linorobot2_description/urdf
RUN rm -rf src/linorobot2/linoroobot2_navigation/maps
COPY linorobot2_description/urdf src/linorobot2/linorobot2_description/urdf
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    colcon build --packages-select linorobot2_description

ENTRYPOINT ["/entrypoint.sh"]

FROM base AS hardware

RUN sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO    
RUN sudo apt-get install -y python3-vcstool build-essential

WORKDIR /root/linorobot2_ws
RUN git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN rosdep install --from-path src --ignore-src -y --skip-keys micro_ros_agent
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    colcon build --packages-select micro_ros_setup

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    ros2 run micro_ros_setup build_agent.sh

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

#install sensors
WORKDIR /root/linorobot2_ws/src/linorobot2/docker
ARG ROBOT_BASE=2wd
ARG LASER_SENSOR=
COPY docker/install_sensors.bash .
RUN /bin/bash install_sensors.bash $ROBOT_BASE $LASER_SENSOR $DEPTH_SENSOR $ROS_DISTRO
WORKDIR /root/linorobot2_ws
RUN rm -rf src/linorobot2/linorobot2_description/urdf
RUN rm -rf src/linorobot2/linoroobot2_navigation/maps
COPY linorobot2_description/urdf src/linorobot2/linorobot2_description/urdf
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    colcon build --packages-select linorobot2_description
RUN apt-get -y install udev
RUN echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules
RUN echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules
RUN echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules
ENTRYPOINT ["/entrypoint.sh"]

FROM hardware AS build

RUN sudo apt-get install -y python3-venv

#Install Platformio
#---------------------
RUN wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
RUN python3 get-platformio.py

ENV PATH=${PATH}:/root/.platformio/penv/bin

WORKDIR /root/linorobot2_ws

RUN git clone https://github.com/${BASE_GIT_LINOROBOT2}/linorobot2_hardware.git src/linorobot2_hardware
RUN cd src/linorobot2_hardware/firmware
WORKDIR /root/linorobot2_ws/src/linorobot2_hardware/firmware
RUN pio run -e teensy40

RUN apt-get install -y avrdude

ENTRYPOINT ["/entrypoint.sh"]

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as rosbridge

# Install Rosbridge
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rosbridge-server

# Set up the entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as foxglovebridge

# Install Rosbridge
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-foxglove-bridge

# Set up the entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

