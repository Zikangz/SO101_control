cmake_minimum_required(VERSION 3.0.2)
project(so101_ros1_bridge)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/so101_driver_node.py
  scripts/so101_control_cli.py
  scripts/so101_kinematics_node.py
  scripts/so101_servo_node.py
  scripts/so101_safe_sequence.py
  scripts/so101_precision_check.py
  scripts/so101_sine_test.py
  scripts/so101_ee_sine_test.py
  scripts/so101_validate_ee_trajectory.py
  scripts/so101_servo_watch.py
  scripts/so101_aerial_state_node.py
  scripts/so101_aerial_velocity_gate.py
  scripts/so101_load_compensation_observer.py
  scripts/so101_low_frequency_bias_observer.py
  scripts/so101_analyze_csv.py
  scripts/so101_follow_joint_trajectory_server.py
  scripts/so101_plot_ee_trajectory.py
  scripts/so101_fit_phase_z_compensation.py
  scripts/so101_fit_backlash_compensation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
