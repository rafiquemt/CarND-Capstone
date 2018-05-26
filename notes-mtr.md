# Some notes

`roscore` master node

`rostopic info <topicName>` get info on a topic
`rostopic list`

Message info
`rosmsg list`

`rosed` can be used to edit a message file / file in a package without full path
so `rosed geometry_msgs Twist.msg`

## Catkin
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

make
```
cd ~/catkin_ws
catkin_make
```

Note `devel/setup.bash` must be sourced before using workspace

`catkin_make` must always be run from catkin_ws folder

## ROSLaunch
ROS launch automatically start stuff

`roslaunch simple_arm robot_spawn.launch`

Note that simple_arm must have a `robot_spawn.lunch` file present

Use rosdep to check for missing dependencies

`rosdep check simple_arm`

Creating packages

`catkin_create_pkg first_package`

Ros wiki link: http://wiki.ros.org/

rospy, publishing messages
http://docs.ros.org/kinetic/api/rospy/html/rospy.topics.Publisher-class.html

*** Make sure to run `source devel/setup.bash` after any `catkin_make`

How to create message
http://wiki.ros.org/msg

How to create srv file
http://wiki.ros.org/srv



Viewing camera images

`rqt_image_view /rgb_camera/image_raw`
associated rqt tools: http://wiki.ros.org/rqt

Note: rosservice call can tab-complete the request message, so that you don’t have to worry about writing it out by hand. Also, be sure to include a line break between the two joint parameters.

rosparam set /arm_mover/max_joint_2_angle 1.57

if you're adding a new script and want it to run as a node
```xml
  <!-- The look away node -->
  <node name="look_away" type="look_away" pkg="simple_arm"/>
``` 

## Logging
By default all logging messages for a node are written to the node's log file which can be found in ~/.ros/log or ROS_ROOT/log . If roscore is running, you can use roscd to find log file directory by opening a new terminal window and typing:

https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/0bf289a5-8890-48f4-918a-bab4f49e0019/concepts/828147cb-cf85-4d12-94b5-a18fd70f9d23

more how to go through logging files

Overview notes:
https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/0bf289a5-8890-48f4-918a-bab4f49e0019/concepts/828147cb-cf85-4d12-94b5-a18fd70f9d23

