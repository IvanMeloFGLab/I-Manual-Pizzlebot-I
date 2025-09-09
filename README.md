# |Manual-Pizzlebot|
Instala ROS2 Jazzy, HAILO TPU, RPI Camara V2 en Ubuntu 25.04 para Raspberry Pi 5.

## Actualizar sistema.
```
sudo apt update && sudo apt upgrade -y
```

## Instalar dependencias.

```
sudo apt install -y git make wget python3-pip
```

## Actualizar PIP.
```
sudo python3 -m pip install --upgrade --break-system-packages pip setuptools wheel
```
Se debería ver algo como:

![alt text](images/uppy.png "UPPY")
---
## Instalación de ROS2 Jazzy Jalisco.

Para mas info referir a la documentación original.

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

*** Instalación no oficial, compilando el código fuente en Ubuntu 25.04 ***

Intalación de dependencias de ROS2.
```
sudo apt install -y libbullet-dev libasio-dev libtinyxml2-dev libssl-dev libyaml-dev libopencv-dev libboost-python-dev\
  libeigen3-dev libboost-all-dev libx11-dev libxext-dev libgl1-mesa-dev libglu1-mesa-dev locales liburdfdom-headers-dev
```
Asegúrate de que tu configuración regional admite UTF-8.
```
locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=e_US.UTF-8
export LANG=en_US.UTF-8
locale
```
Intalación de dependencias en python de ROS2.
```
sudo python3 -m pip install --break-system-packages catkin_pkg vcstool rosdistro rosdep rosinstall-generator colcon-common-extensions opencv-python lark pybind11
```
Se debería ver algo como:

![alt text](images/deppy.png "UPPY")

Agrando al PATH.
```
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```
Crear un espacio de trabajo para los paquetes internos de ROS2.
```
mkdir -p ~/ros2_internal_ws/src
cd ~/ros2_internal_ws
```
Descargar los archivos fuente de ROS2.
```
wget https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
vcs import src < ros2.repos
cd src/
git clone -b rolling https://github.com/ros-perception/vision_opencv.git
cd ..
```
Se debería ver algo como:

![alt text](images/rosget.png "UPPY")

Resolver dependencias de ROS2.
```
sudo -H rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
  --skip-keys "urdfdom_headers python3-catkin-pkg-modules rti-connext-dds-6.0.1 python3-rosdistro-modules python3-vcstool cv_bridge libboost-python"
```
Compilación de archivos fuente de ROS2.
*** ADVERTENCIA: Este procedimiento tarda alrededor de 3 horas. ***
```
cd ~/ros2_internal_ws
colcon build --symlink-install
```
Obtener instalación de ROS2.
```
source ~/ros2_internal_ws/install/setup.bash
```
Configuración de ROS2 y adición de alias para atajos.
```
echo "source ~/ros2_internal_ws/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_internal_ws/install/"
echo "alias cb='cd ~/ros2_ws; colcon build && source install/setup.bash'"
echo "alias so='cd ~/ros2_ws; source install/setup.bash'"
```
Crear un espacio de trabajo para ROS2.
```
cd && mkdir ros2_ws
source ~/.bashrc
```

---
## Instalación de Micro-ros

Para mas info referir a la documentación original.

https://micro.ros.org/docs/tutorials/core/first_application_linux/

Crear un espacio de trabajo y descargar las herramientas micro-ROS.
```
cd && mkdir uros
cd uros
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
Actualizar dependencias utilizando rosdep.
```
rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src -y
```
Crear herramientas micro-ROS y cargarlas.
```
colcon build
source install/local_setup.bash
```
Descargar el agente de micro-ROS.
```
ros2 run micro_ros_setup create_agent_ws.sh
```
Crear el agente de micro-ROS.
```
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
---
## Cámara RPI V2 configuración y pruebas.

Siguiendo procedimiento de:

https://askubuntu.com/questions/1542652/getting-rpicam-tools-rpicam-apps-working-on-ubuntu-22-04-lts-for-the-raspber

Basado en:

https://github.com/raspberrypi/libcamera

https://github.com/raspberrypi/rpicam-apps.git

https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera#Working_with_Raspberry_Pi_5_.28libcamera.29:%7E:text=Camera%20Documentation.-,Working%20with%20Raspberry%20Pi%205%20(libcamera),-Bookworm%20will%20not

```
sudo apt install clang meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 \
  openssl libdw-dev libunwind-dev libudev-dev libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libpython3-dev pybind11-dev libevent-dev libtiff-dev qt6-base-dev qt6-tools-dev-tools liblttng-ust-dev \
  python3-jinja2 lttng-tools libexif-dev libjpeg-dev pybind11-dev libevent-dev libgtest-dev abi-compliance-checker -y
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
sudo sed -i '/app.ShowPreview(completed_request, app.VideoStream());/d' ~/rpicam-apps/apps/rpicam_vid.cpp
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
sudo reboot
```

```
rpicam-hello --list-cameras
```

```
rpicam-hello -camera <camera_number>
```
---
## Crear cámara virtual y pipeline para uso en ROS2 a travéz de OpenCV.

Para mas info referir a la documentación original.

https://github.com/umlaeute/v4l2loopback

https://wiki.archlinux.org/title/V4l2loopback

Descargar y compilar v4l2loopback desde fuente.
```
git clone https://github.com/umlaeute/v4l2loopback
cd v4l2loopback
```
Instalar v4l2loopback.
```
make
sudo make install
sudo depmod -a
```
Crear cámara virtual, video_nr=<número de dispocitivo virtual> ej.100 para evitar conflictos con camaras reales,
card_label=<Nombre de cámara virtual>, exclusive_caps=1 modo donde el dispositivo solo reporta capacidades de salida.
```
sudo modprobe v4l2loopback video_nr=100 card_label="RPiCam Virtual" exclusive_caps=1
```

```
rpicam-vid -t 0 --codec mjpeg --inline -o - | ffmpeg -f mjpeg -i - -f v4l2 -pix_fmt yuyv422 /dev/video100
```
----
## Instalación de HAILO8 TPU

https://ubuntu.com/blog/hackers-guide-to-the-raspberry-pi-ai-kit-on-ubuntu

Intalación de dependencias de Hailo8.
```
sudo apt install -y dkms linux-headers-$(uname -r)
```
Clonar el repositorio hailort-drivers y cambiar a la rama hailo8.
```
git clone --branch hailo8 https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers
```
Compilar el controlador.
```
cd linux/pcie
make all
sudo make install
```
Descargar el firmware y cargarlo.
```
./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo cp hailo8_fw.4.22.0.bin /lib/firmware/hailo/hailo8_fw.bin
```
Revisar si es detectado.
```
ls -l /dev/hailo*
sudo dmesg | grep -i hailo
```
Descargar HailoRT para comprobar funcionamiento del TPU.
```
cd && git clone https://github.com/hailo-ai/hailort.git
cd hailort
git checkout hailo8
```
Installar HailoRT.
```
cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --config release --target install
```
Comprobar funcionamiento del TPU.
```
hailortcli fw-control identify
hailortcli device test
```
