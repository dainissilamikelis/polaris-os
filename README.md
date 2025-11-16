# Documents

See description diagrams in folder Docs.

# Polaris UGV Setup

1. GroundControl host must have IP address: 172.25.64.194
2. Polaris UGV host must have IP address: 172.25.65.12

## Requirements

1. Check ZED box system:
```bash
lsb_release –a
dpkg-query -W nvidia-l4t-core
nvidia-smi
apt-cache show nvidia-jetpack
```
The output must be:
Orion NX 16GB: is R36 (release), REVISION: 3.0 - > JetPack 6.0 & L4T Version 36.3.0

2. Install ROS2 Humble:
[instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)


3. Install ZED StereoLabs SDK:
[more infor](https://www.stereolabs.com/docs/embedded/zed-box-orin)
```bash
mkdir -p pol_ws/downloads && cd pol_ws/downloads
wget -O ZED_SDK_Tegra_L4T36.3_v5.0.7.zstd.run "https://download.stereolabs.com/zedsdk/5.0/l4t36.3/jetsons_gl=1*1bezr7q*_gcl_au*MTIwNTg0MDg0Mi4xNzU2Mjg3Mzcx"
chmod +x ./ZED_SDK_Tegra_L4T36.3_v5.0.7.zstd.run
ZED_SDK_Tegra_L4T36.3_v5.0.7.zstd.run
```

4. Install [ZEDLink drivers](https://www.stereolabs.com/docs/embedded/zed-link/install-the-drivers) if not installed:

download deb file: stereolabs-zedbox-duo_1.3.1-LI-MAX96712-all-ZEDBOX-L4T36.3.0_arm64.deb and:

```bash
sudo dpkg -i ./stereolabs-zedbox-duo_1.3.1-LI-MAX96712-all-ZEDBOX-L4T36.3.0_arm64.deb
sudo reboot
sudo dmesg | grep zedx
```
5. Install ZED gstreamer libraries ([more info](https://github.com/stereolabs/zed-gstreamer)):
```bash
git clone https://github.com/stereolabs/zed-gstreamer.git
cd zed-gstreamer
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
sudo apt update
sudo apt install libgstrtspserver-1.0-dev
```

and check installed plugins:

```bash
gst-inspect-1.0 zedsrc
gst-inspect-1.0 zedxonesrc
gst-inspect-1.0 zeddemux
gst-inspect-1.0 zeddatamux
gst-inspect-1.0 zeddatacsvsink
gst-inspect-1.0 zedodoverlay
```
6. Connect and test cameras on the UGV
```bash
sudo systemctl restart zed_x_daemon
ZED_Diagnostic -c
```

## UGV Setup

1. Create ROS2 working space directory:

```bash
cd ~/pol_ws
mkdir src
colcon build
```

2. Init the working space:
```bash
colcon build --symlink-install
source install/setup.bash
```

3. Clone the UGV repository:
```bash
cd src
git clone git@github.com:dainissilamikelis/polaris-os.git
cd polaris-os
git submodule update --init --recursive
```

4. Build mavlink libraries:
```bash
cd polaris-ground-control/mavlink
cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=~/gc_ws/install/mavlink -DMAVLINK_DIALECT=common -DMAVLINK_VERSION=2.0
cmake --build build --target install
touch COLCON_IGNORE
```

5. Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-can-msgs
```
```bash
cd ~/pol_ws/src
git clone https://bitbucket.org/DataspeedInc/dataspeed_can.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select dataspeed_can_msg_filters
```
```bash
cd src
git clone https://github.com/autowarefoundation/ros2_socketcan.git
cd ..
rosdep install --from-paths src --ignore-src -r –y
colcon build --packages-select ros2_socketcan ros2_socketcan_msgs
```

6. Setup CAN-USB DataSpeed tool device:

***set correct ATTR{idVendor} and ATTR{idProduct} values before executing***

```bash
lsusb
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="6923", ATTR{idProduct}=="0112", MODE="0666"' | sudo tee /etc/udev/rules.d/99-dataspeed-can.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls /dev | grep can
modprobe can
modprobe can_raw
modprobe can_dev
ip link show
```

7. Setup GNSS reciever [more info](https://www.stereolabs.com/docs/global-localization/using-gnss-linux)

Setup other tools:
```bash
sudo apt install libgps-dev
```
check the device is working:

```bash
cgps -s
```

8. Build ROS2 packages:

```bash
cd ~/pol_ws
source install/setup.bash
sudo rosdep init
rosdep update
source install/setup.bash
colcon build --symlink-install
```

## Set UGV software auto start

1. Edit ~/pol_ws/ polaris-os/scripts/polaris.service to set correct path of run.sh, user name, and working directory.
2. Edit ~/pol_ws/ polaris-os/scripts/run.sh to set CG IP address
3. Setup service files:
```bash
sudo cp ~/pol_ws/src/polaris-os/scripts/polaris.service /etc/systemd/system
chown user:user /home/user/pol_ws/src/polaris-os/scripts/run.sh
chmod +x /home/user/pol_ws/src/polaris-os/scripts/run.sh
```
4. Apply service config:
```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable polaris.service
sudo systemctl start polaris.service
```
5. Check service status:
```bash
sudo systemctl status polaris.service
# or
journalctl -u polaris.service
```

## Run UGV software manually (if necessary)

1. Stop polaris.service:
```bash
sudo systemctl stop polaris.service
```

2. In terminal 1:
```bash
cd ~/pol_ws
source install/setup.bash
ros2 launch polaris_joystick joystick.launch.xml
```

3. In terminal 2:
```bash
cd ~/pol_ws
source install/setup.bash
ros2 launch mavros_commutator run.xml
```

4. In terminal 3:
```bash
cd ~/pol_ws
source install/setup.bash
./build/ZED_Streaming_Sender/ZED_Streaming_Sender 5002
```

5. In terminal 4:
```bash
cd ~/pol_ws
source install/setup.bash
./build/ZED_One_live/ZED_One_streaming_sender 5004
```