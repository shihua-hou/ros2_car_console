cmake_minimum_required(VERSION 3.8)
project(tagdocking)

# ── Dependencies ──────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# ── Generate action interface ─────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dock.action"
  DEPENDENCIES action_msgs
)

# ── Install Python package ────────────────────────────────────────
# All modules under tagdocking/ (including base_adapter/)
install(
  DIRECTORY tagdocking
  DESTINATION local/lib/python3.10/dist-packages
  PATTERN "__pycache__" EXCLUDE
)

# ── Install entry-point scripts ───────────────────────────────────
install(PROGRAMS
  scripts/docking_node
  scripts/camera_info_bridge
  scripts/test_turn_angle
  scripts/test_jog_distance
  scripts/test_apriltag
  scripts/calibrate_camera
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch and config ─────────────────────────────────────
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
