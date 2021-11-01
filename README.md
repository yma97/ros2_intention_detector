# ros2_intention_detector
JUST_THINK Optional Semester Project

cd ros2_foxy/src/
. ../ros2-linux/setup.bash
. install/setup.bash

colcon build --packages-select ros2_intention_detector


ros2 run ros2_intention_detector listener


ros2 topic pub /intention std_msgs/String "data: Connect"