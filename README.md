Run 4 nodes in 4 terminals
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 color_module.enable:=false


ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.0.10

ros2 run tf2_ros static_transform_publisher   0.052364 0.060511 0.023395   0.012047 0.003220 0.999053 0.041678   end_effector_link camera_color_optical_frame


ros2 run tf2_ros static_transform_publisher   0.0 0.0 0.0 0.0 0.0 0.0 1.0   end_effector_link camera_link

Then in this directory, run
python3 detection.py

python3 pick_place.py
