# ouijabot
Omnidirectional robot developed at Multi-robot Systems Lab, Stanford University

### Robot side: use package "ouijabot"
- ssh into the robot
- Check /etc/hosts to see if the master IP has been added
- Edit ~/.bahsrc so that the following two lines reflect the robot's own IP as well as the master IP
```
export ROS_HOSTNAME=ouijabot1 # robot's own hostname/IP
export ROS_MASTER_URI=http://msl-Lenovo:11311 # PC hostname/IP
```
- In **ouijabot/launch/ouijabot.launch**, edit the number in **group ns="ouijabot1"** if you need to change the index of the robot
- catkin_make the package
- `source ./catkin_ws/devel/setup.bash`
- Fire `roslaunch ouijabot ouijabot.launch` and ready to go!
- **Important**: please poweroff the Raspberry Pi (run `sudo poweroff` in ssh)  before turning off the Ouijabot


### Laptop side: use package "ouijabot_telop_cpp"
- Check /etc/hosts and ~/.bashrc like we did for the robot
- In **ouijabot_telop_cpp/launch/telop.launch**, change the index in **remap from="vel_out" to="/ouijabot1/cmd_vel"** if need to control different ouijabots
- Fire `roslaunch ouijabot_telop_cpp telop.launch` and enjoy driving the Ouijabot!

### Reference
If you use the robot, please consider citing the following article:

Zijian Wang, Guang Yang, Xuanshuo Su, and Mac Schwager, "OuijaBots: Omnidirectional Robots for Cooperative Object Transport with Rotation Control using No Communication", *Proc. of the International Symposium on Distributed Autonomous Robotics Systems (DARS)*, London, UK, November, 2016
