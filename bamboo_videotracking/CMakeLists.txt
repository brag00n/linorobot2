cmake_minimum_required(VERSION 3.8)
project(bamboo_videotracking)

# PREMIER paquet ament_cmake maison du depot (bamboo_base et bamboo_controler_* sont
# ament_python, bamboo_interfaces ne genere que des messages). Consequence operatoire :
# contrairement a un noeud rclpy lance par chemin absolu sous le bind-mount, celui-ci
# DOIT etre construit -- le service compose fait son `colcon build --packages-select`,
# comme driver.real le fait deja.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bamboo_interfaces REQUIRED)
# OpenCV >= 4.10, compile depuis les sources dans /usr/local par docker/Dockerfile, A COTE
# du 4.5.4 d'apt. Ce n'est pas un confort : le 4.5.4 de Jammy ne sait PAS lire
# face_detection_yunet_2023mar.onnx (mesure, six tailles d'entree essayees) et n'a pas
# cv::TrackerVit. Ce paquet ne dependant pas de cv_bridge, il peut se lier a 4.10 seul --
# bamboo_video, lui, reste sur 4.5.4 seul. Ne JAMAIS melanger les deux dans un processus.
# Les composants demandes explicitement pour que l'absence d'un module echoue ICI, a
# la configuration, et non au link avec un symbole manquant :
#   video     -> cv::TrackerMIL / cv::TrackerVit et cv::KalmanFilter
#   objdetect -> cv::FaceDetectorYN, cv::FaceRecognizerSF
#   imgcodecs -> imdecode / imencode (MJPG entrant, JPEG annote sortant)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video objdetect)
# Trace OBLIGATOIRE : deux OpenCV coexistent dans l'image, et un find_package qui retombe
# silencieusement sur /usr/lib/aarch64-linux-gnu/cmake/opencv4 rendrait le build "reussi"
# mais YuNet 2023mar toujours illisible a l'execution. La ligne le prouve.
message(STATUS "bamboo_videotracking : OpenCV ${OpenCV_VERSION} depuis ${OpenCV_DIR}")
if(OpenCV_VERSION VERSION_LESS "4.8")
  message(FATAL_ERROR
    "OpenCV ${OpenCV_VERSION} trouve dans ${OpenCV_DIR} : trop ancien pour "
    "face_detection_yunet_2023mar.onnx (>= 4.8 requis). La couche source du Dockerfile "
    "n'a pas ete construite, ou CMake a prefere l'OpenCV d'apt.")
endif()

# --- bibliotheque de portage partagee par les composables ---------------------
# La detection, la machine a etats de verrou et le predicteur sont COMMUNS a
# plusieurs noeuds : ils vivent dans une bibliotheque, pas recopies par noeud.
add_library(bamboo_vt_core SHARED
  src/face_detection.cpp
  src/kalman_predictor.cpp
  src/gallery.cpp
)
target_include_directories(bamboo_vt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bamboo_vt_core ${OpenCV_LIBS})

# --- les quatre composables du chemin chaud -----------------------------------
# UN SEUL processus (component_container_mt) : c'est la seule facon d'avoir la
# cv::Mat partagee PAR POINTEUR. rclpy n'a pas l'intra-process zero-copy, et la
# memoire partagee DDS est coupee par l'entrypoint (shm_off.xml) -- une trame brute
# qui traverse une frontiere de processus coute donc plein tarif (921 Ko en rgb8).
set(BAMBOO_VT_COMPONENTS tracking_node facerecog_node overlay_node servocam_node)
foreach(comp ${BAMBOO_VT_COMPONENTS})
  add_library(${comp} SHARED src/${comp}.cpp)
  target_include_directories(${comp} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(${comp} bamboo_vt_core ${OpenCV_LIBS})
  ament_target_dependencies(${comp}
    rclcpp rclcpp_components std_msgs sensor_msgs bamboo_interfaces)
endforeach()

rclcpp_components_register_node(tracking_node
  PLUGIN "bamboo_videotracking::TrackingNode" EXECUTABLE tracking_node_exe)
rclcpp_components_register_node(facerecog_node
  PLUGIN "bamboo_videotracking::FaceRecogNode" EXECUTABLE facerecog_node_exe)
rclcpp_components_register_node(overlay_node
  PLUGIN "bamboo_videotracking::OverlayNode" EXECUTABLE overlay_node_exe)
rclcpp_components_register_node(servocam_node
  PLUGIN "bamboo_videotracking::ServoCamNode" EXECUTABLE servocam_node_exe)

install(TARGETS bamboo_vt_core ${BAMBOO_VT_COMPONENTS}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# --- volet Python : apprentissage et banc, hors chemin temps reel -------------
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  bamboo_videotracking/face_train_node.py
  bench/replay_bench.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
