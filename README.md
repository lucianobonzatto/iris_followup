# iris_followup

## iris simulation

editar o arquivo em 

```
src/Firmware/Tools/sitl_run.sh

src/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf

src/Firmware/Tools/sitl_gazebo/models/fpv_cam/fpv_cam.sdf
```

https://github.com/piradata/wpg

https://github.com/piradata/PX4-Autopilot


### install

```
sudo apt install liblzma-dev lzma
pip3 install toml numpy packaging jinja2
sudo apt install libgstreamer1.0-dev
sudo apt install genromfs ninja-build exiftool astyle
sudo apt install libgstreamer-plugins-base1.0-dev
```
```
mkdir -p ~/src/
cd ~/src/
git clone https://github.com/piradata/PX4-Autopilot.git Firmware
cd Firmware/
make px4_sitl gazebo_iris
```

### run
```
cd ~/src/Firmware/
make px4_sitl gazebo_iris
roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'
```
