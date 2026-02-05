# install instruction

## install dependancy

```bash
cd noetic_ubuntu20
sudo dpkg -i libtum-ics-*
sudo dpkg -i libtumtools-*
sudo dpkg -i ros-*
catkin build -DTUM_ICS_USE_QT5=1
```

## setup command

**Note**: run in dev container!

```bash
rosmaster_ur10()
{
export ROS_MASTER_URI=http://192.168.1.3:11311
export ROS_IP=192.168.1.3
}
rosmaster_local()
{
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
}
```
