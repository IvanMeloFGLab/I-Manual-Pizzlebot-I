# |Manual-Pizzlebot|
Instala ROS2 Jazzy, HAILO TPU, RPI Camara V2

## Install Depenedncies

```
sudo apt-get install python3-pip -y
```

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
sudo apt install software-properties-common -y
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

## Micro-ros install

```
source /opt/ros/$ROS_DISTRO/setup.bash
```

```
mkdir uros
cd uros
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

```
rosdep init
rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src -y
```

```
colcon build
source install/local_setup.bash
```

```
ros2 run micro_ros_setup create_agent_ws.sh
```

```
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## RPI Camara V2 setup && test

```
sudo apt install clang meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 openssl -y
```

```
sudo apt install libdw-dev libunwind-dev libudev-dev libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libpython3-dev pybind11-dev libevent-dev libtiff-dev qt6-base-dev qt6-tools-dev-tools liblttng-ust-dev python3-jinja2 lttng-tools libexif-dev libjpeg-dev pybind11-dev libevent-dev libgtest-dev abi-compliance-checker -y
```

```
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
ninja -C build install
sudo ninja -C build install
```

```
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps/
sudo apt install cmake libboost-program-options-dev libdrm-dev libexif-dev -y
sudo apt install ffmpeg libavcodec-extra libavcodec-dev libavdevice-dev libpng-dev libpng-tools libepoxy-dev -y
sudo apt install qt5-qmake qtmultimedia5-dev -y
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
meson compile -C build
sudo meson install -C build
```

```
sudo ldconfig
rpicam-still --version
```

```
sudo tee -a /boot/firmware/config.txt <<EOF
dtoverlay=imx219,cam0
dtoverlay=imx219,cam1
EOF
```

```
rpicam-hello --list-cameras
```

```

```

```

```

```

```
