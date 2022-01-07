# “Help a humanoid robot understand my verbalised intentions!”
### ros2_intention_detector - EPFL CHILI Lab - [JUST_THINK](https://www.epfl.ch/labs/chili/index-html/research/animatas/justhink/) Optional Semester Project 

#### Motivation: 
The aim of this project is to endow a humanoid robot Reachy with the ability to understand the verbalized intentions of the human, in order to enhance the interactions.

#### Methods: 
Basically, users have five kinds of intentions in this process: connect, agree, disagree, clear, and submit. To enable Reachy to understand users’ intentions correctly, we developed two scripts: speech_recogniser to do automatic speech recognition using Google Speech Recognition API (v2) with a feedback system indicating when the user should speak, and intention_interpreter to detect user’s intention through keyword analysis and follow-up questions. In this project, an iterative design has been used to continuously improve the smooth interaction, including an initial survey of brainstorming for a possible way to express the same intention, and four rounds of user tests to do the activity with/without visual/audio feedback system and with/without the presence of the robot. 

#### Results: 
With four metrics detection rate, average steps taken per intention, average followups used per intention, average time used per intention, we can conclude that the feedback system can improve the intention detection performance, especially for more complex intentions such as connect, where the audio feedback performs better than the visual feedback that distracts user’s attention. Also, without adding additional stress, users expect to have more interaction with the robot, which makes them feel more involved in the activity. 

#### Pre-requrirement: 
Have [JustThink_World](https://github.com/utku-norman/justhink_world) installed;

#### Installation:
```
git clone https://github.com/yma97/ros2_intention_detector.git
cd ros2_foxy/src/
. ../ros2-linux/setup.bash
. install/setup.bash
```
#### Install dependencies and build the package:
```
rosdep install --from-paths src --ignore-src --rosdistro foxy -y -r
colcon build --packages-select ros2_intention_detector
```
#### Run the node:
```
ros2 run ros2_intention_detector detector
ros2 run ros2_intention_detector listener
ros2 topic pub /intention std_msgs/String "data: Connect"
```
#### If there is error related to usb access permission when using LED lights on Reachy:
```
sudo udevadm control --reload
sudo udevadm trigger
```
