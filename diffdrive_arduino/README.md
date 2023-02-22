# diffdrive_arduino


This node is designed to provide an interface between a `diff_drive_controller` from `ros_control` and an Arduino running firmware from `ros_arduino_bridge`.

sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


