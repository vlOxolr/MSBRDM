# install instruction

**Note**: run all commands in container!

## install dependancy

```bash
cd noetic_ubuntu20

sudo dpkg -i libtum-ics-*
sudo dpkg -i libtumtools-* #check error msg
# sudo dpkg -i tum-ics-libconfig1_0.0.0-1_all.deb
sudo dpkg -i ros-*

catkin build -DTUM_ICS_USE_QT5=1
```

## setup command

Modify .bashrc

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

## Guide for merging to develop

```bash
# last commit
git add .
git commit -m "last commit"

# merge
git switch develop
git pull
git merge <your personal branch name>
git push # not neccessary, but recommended
git switch <your personal branch name>
git merge develop
```