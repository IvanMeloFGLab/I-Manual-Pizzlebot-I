# |Manual-Pizzlebot|
Instala ROS2 Jazzy, HAILO TPU, RPI Camara V2

## ROS2 Jazzy Jalisco install

Para mas info referir a la documentación original.
https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html

```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
```

```
sudo apt update && sudo apt install ros-dev-tools
```

```
sudo apt update
```

```
sudo apt upgrade
```

```
sudo apt install ros-jazzy-desktop
```

```
sudo apt install ros-jazzy-ros-base
```

```
source /opt/ros/jazzy/setup.bash
```

```
cd
mkdir ros2_ws
```


```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/jazzy/"
echo "alias cb='cd ~/ros2_ws; colcon build && source install/setup.bash'"
echo "alias cb='cd ~/ros2_ws; source install/setup.bash'"
```

```

```
