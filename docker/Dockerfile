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
# La synchro Windows->RPi peut perdre le bit +x ; COPY preserve le mode source -> on le force.
RUN sed -i "s/rosdistro/$ROS_DISTRO/" /entrypoint.sh && chmod +x /entrypoint.sh
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

# Camera USB (webcam UVC 05a3:9331, MJPG/YUYV/H264) : v4l2_camera publie /image_raw,
# web_video_server diffuse en HTTP (:8080), image_transport_plugins fournit le transport
# /image_raw/compressed (MJPEG). Layer final du stage hardware -> rebuild rapide.
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-v4l2-camera \
    ros-$ROS_DISTRO-web-video-server \
    ros-$ROS_DISTRO-image-transport-plugins

# --- Metrique FPS video (camera_fps_node.py) : AUCUNE dep a installer. Le noeud n'utilise que
#     la stdlib Python (urllib, threading) + rclpy/std_msgs/sensor_msgs deja presents. Lance
#     par CHEMIN ABSOLU depuis le bind-mount (fichier neuf, pas de symlink dans install/share)
#     par camera.launch.py (arg enable_fps_metric, defaut true). Publie /camera/capture_fps et
#     /camera/stream_fps (Float32, visibles via MCP). Voir docs/bamboo4WD_V4_WSEsp32.md.
#
# --- H.264 distant a 30 fps (etape 1, FAIT) : hors de cette image. Service compose camera.h264
#     = image officielle bluenviron/mediamtx:latest-ffmpeg + docker/mediamtx.yml. ffmpeg recopie
#     le H.264 materiel de la webcam (-c:v copy, zero re-encodage RPi) -> RTSP/HLS/WebRTC.
#     Rien a ajouter ici (MediaMTX embarque son propre ffmpeg).
#
# --- MJPG->ROS a 30 fps (etape 2, IMPLEMENTE - build+validation live sur RPi a faire) :
#     publier le MJPG MATERIEL en /image_raw/compressed
#     SANS le decoder, via gscam2 (pipeline GStreamer image_encoding=jpeg -> copie le buffer
#     JPEG). web_video_server le ressert en type=ros_compressed (ni decode ni re-encode) ->
#     ~30fps a coup CPU quasi nul, vs ~12fps du chemin v4l2 (camera.real). Service compose
#     camera.mjpg + camera_mjpg.launch.py. gscam2 n'est pas un paquet apt -> build colcon.
#     v4l2src (capture MJPG) vient de gstreamer1.0-plugins-good, absent des deps package.xml
#     de gscam2 -> on l'apt-installe explicitement (rosdep ne le tirerait pas).
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-tools
RUN git clone https://github.com/clydemcqueen/gscam2.git src/gscam2
RUN apt-get update && rosdep install --from-path src/gscam2 --ignore-src -y
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/linorobot2_ws/install/setup.bash && \
    colcon build --packages-select gscam2

# pyserial pour le driver ESP32 (pont MAVLink) : robot_control (codec) est du pur Python
# monte via PYTHONPATH, sa seule dep runtime est pyserial. python3-serial = paquet Debian
# (l'image n'a pas pip). Layer final -> rebuild rapide.
RUN apt-get update && apt-get install -y python3-serial

# --- Vision pour bambooSTM32YB (groupe bamboo_videotracking) --------------------------
# Trois besoins distincts, d'ou trois familles de paquets et NON un seul "opencv" :
#   - libopencv-dev : les EN-TETES C++. Le chemin chaud (detection YuNet, suivi MIL,
#     reconnaissance SFace, Kalman) est du C++ rclcpp, parce que le zero-copy
#     intra-processus n'existe QUE dans rclcpp : une trame rgb8 640x480 pese 921 Ko et
#     l'entrypoint coupe la memoire partagee DDS (shm_off.xml), donc chaque saut de
#     processus se paie plein tarif. Le portage est bon marche : YuNet, SFace, MIL, ViT et
#     Kalman SONT des classes OpenCV C++ (cv::FaceDetectorYN, cv::FaceRecognizerSF,
#     cv::TrackerMIL, cv::TrackerVit, cv::KalmanFilter) -- le prototype Python n'est qu'un
#     binding par-dessus, et les .onnx sont les memes.
#   - cv-bridge / vision-opencv : conversion sensor_msgs <-> cv::Mat, cote C++ ET Python.
#   - python3-opencv : pour le SEUL noeud d'apprentissage (enrolement SFace), qui n'est pas
#     temps reel et reste en rclpy, recopie du prototype.
# image-transport est deja tire par image-transport-plugins ci-dessus ; on le declare quand
# meme, une dependance implicite qui disparait en amont est une panne difficile a lire.
#
# ATTENTION, A VERIFIER DANS LE CONTENEUR AVANT D'ECRIRE LE C++ :
#     pkg-config --modversion opencv4
# Humble/Jammy livre OpenCV 4.5.4. cv::FaceDetectorYN, cv::FaceRecognizerSF et
# cv::TrackerMIL y sont, mais cv::TrackerVit n'est apparu qu'en 4.7. Si 4.5.4 est confirme :
# le suivi se limite a MIL (le prototype sait deja retomber) et le mode vit reste DECLARE
# mais repond "indisponible" -- recompiler OpenCV sur un RPi4 coute des heures pour un gain
# non demontre. Pas de pip : l'image n'en a pas et on n'en ajoute pas.
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-image-transport

# --- Manette (groupe bamboo_control) -------------------------------------------------
# Installe ICI et pas au lot V5 : un apt-get dans cette image coute un rebuild complet sur
# RPi4, autant n'en payer qu'un. Ces paquets ne sont declares qu'en exec_depend de
# linorobot2_bringup/package.xml, et rosdep tourne contre un clone DISTANT, pas contre
# l'arbre de travail -> il ne les tire pas. joystick fournit jstest, qui sert a relever les
# index d'axes : ils CHANGENT entre le mode XInput et le mode DirectInput de la manette.
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-teleop-twist-joy \
    joystick

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
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}-ros-base as foxglovebridge

# Install Rosbridge
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-foxglove-bridge

# Set up the entrypoint
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

