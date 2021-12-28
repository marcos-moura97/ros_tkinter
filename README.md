# ROSTkinter

ROS node that runs and show the information of your project in a GUI based on [Tkinter](https://docs.python.org/3/library/tkinter.html). This project is developed based on [ROSBoard project](https://github.com/dheera/rosboard).

**ROS1/ROS2 compatible.** This package will work in either ROS version.

**Light weight.** Doesn't depending on much. Consumes extremely little resources when it's not actually being used.

<h1 align="center">
    <img alt="TracSense" ttle="TracSense" src="https://drive.google.com/uc?export=view&id=1IWiIiZ1ZM2bTzMlpvtfrGycIdfhbE7ii" />
</h1>

## Prerequisites


```
apt-get install python-tk # ubuntu
pip install tkinter #windows
```


## Running it the easy way (without installing it into a workspace)

```
source /opt/ros/YOUR_ROS1_OR_ROS2_DISTRO/setup.bash
./run
```

Then is just check your nodes information.

## Installing it as a ROS package

This ROS package should work in either ROS1 or ROS2. Simply drop it into your `catkin_ws/src/` or `colcon_ws/src/` and it should just work.

For ROS 1, run it with `rosrun ros_tkinter ros_tkinter_node` or put it in your launch file.

For ROS 2, run it with `ros2 run ros_tkinter ros_tkinter_node` or put it in your launch file.


## Future Features
- Enable multiple subscriptions
- Enable show Images
- Organize the Structure
