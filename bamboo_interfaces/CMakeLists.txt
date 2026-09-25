cmake_minimum_required(VERSION 3.5)
project(bamboo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # --- telemetrie des cartes de controle ---
  "msg/WheelRpm.msg"
  "msg/Esp32Status.msg"
  "msg/Stm32Status.msg"
  # --- contrat servo : axes NOMMES, aucune carte nommee ---
  "msg/ServoCmd.msg"
  # --- groupe videotracking ---
  "msg/ModeCmd.msg"
  "msg/TrackState.msg"
  "msg/RecognitionResult.msg"
  "msg/TrackingStats.msg"
  # --- services ---
  "srv/Nudge.srv"
  "srv/RegisterOverlay.srv"
  "srv/RecognizeFile.srv"
  "srv/AcquireFile.srv"
  # --- action (apprentissage : long et annulable) ---
  "action/TrainFaces.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
