# Rosnode for ADIS Imu

## Dependencies

Make sure that there are following python packaged installed:
* serial
* cobs

To install those run:

```bash
sudo pip install serial
sudo pip install cobs
```

## How to get the node running

First open new terminal window and run roscore

After that in new terminal run (This snippet has default name of catkin workspace catkin_ws):

```bash
cd ~/catkin_ws
source devel/setup.bash
catkin_make
roslaunch imu_adis imu_adis.launch
```

