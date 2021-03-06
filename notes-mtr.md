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

## Running with image recognition on simulator

https://discussions.udacity.com/t/steering-works-at-20hz-but-not-50hz/750960
This post suggests lowering the look ahead and rate of waypoint generation

## Traffic light detector

setting up udacity-ami on aws for training

https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/595f35e6-b940-400f-afb2-2015319aa640/lessons/b6de6f99-281f-44e8-a9df-196577204638/concepts/1f496bc5-022b-4bff-b30c-cb3bc6eeae98

Using instructions from https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI/blob/master/README.md
with hints from: 
https://github.com/stefancyliax/CarND-Capstone/blob/master/ros/src/tl_detector/light_classification/frozen_models/training.md

Running training commands
`python object_detection/train.py --pipeline_config_path=config/ssd_inception-traffic-udacity_sim.config --train_dir=data/sim_training_data/sim_data_capture`

For the model to work on carla, it must be exported using model v 1.3
`git checkout -b r1.3 9a811d95c478b062393debd1559df61490b97149`

`python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_inception-traffic-udacity_sim.config --trained_checkpoint_prefix=data/sim_training_data/sim_data_capture/model.ckpt-5000 --output_directory=frozen_models/frozen_sim_ssd_inception/`

Then go back to r1.5
`git checkout r1.5`

For real world images, 

Train:

`python object_detection/train.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --train_dir=data/real_training_data`

Export:

`python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --trained_checkpoint_prefix=data/real_training_data/model.ckpt-5000 --output_directory=frozen_models/frozen_real_ssd_inception/`


### What versions of tensorflow models are required ? 

You'll want an earlier version: r1.5
https://github.com/tensorflow/models/tree/r1.5
Checkout r1.5 branch so that you can run these models against tensorflow v 1.3.

Note that the version of tensorflow is completely independent of the the tensorflow models repo. That's why you need to manually 
sync back to a point in time where tensorflow 1.3 worked with tensorflow/models repo. That's r1.5 branch

Now the issue is that exporting the models must also be done in a compatible way. Turns out only back in October 2017 did the 
tensorflow/model repo export script work with tensorflow. 
So to export, you have to checkout an even older commit.

```
cd ~/tensorflow/models
git checkout 9a811d95c478b062393debd1559df61490b97149
```


