# ros2_intention_detector
### “Help a humanoid robot understand my verbalised intentions!” 
### EPFL CHILI Lab - JUST_THINK Optional Semester Project 

Description: Help Interaction between the world activity and the children. Using Speech recognition to detect user's intention.

Pre-requrirement: Have JustThink World installed;

cd ros2_foxy/src/

. ../ros2-linux/setup.bash

. install/setup.bash

colcon build --packages-select ros2_intention_detector

ros2 run ros2_intention_detector detector
ros2 run ros2_intention_detector listener

ros2 topic pub /intention std_msgs/String "data: Connect"
