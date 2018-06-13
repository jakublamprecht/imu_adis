# Rosnode for ADIS Imu

## Dependencies

Make sure that there are following python packaged installed:
* pyserial
* cobs

To install those run:

```bash
sudo pip install pyserial --upgrade
sudo pip install cobs
```

## How to get the node running

First open new terminal window, navigate to ROS workspace directory and run roscore:

```bash
source devel/setup.bash
roscore
```

After that in new terminal tab navigate to ROS workspace directory and run:

```bash
source devel/setup.bash
catkin_make
roslaunch imu_adis imu_adis.launch
```

## Information

Data is published to `/adis/imu` topic

## Known issues

Sometimes while launching the node an error that says `no module named threaded found`. To fix that make sure that you install `pyserial` with --upgrade flag

```bash
sudo pip install pyserial --upgrade
```
