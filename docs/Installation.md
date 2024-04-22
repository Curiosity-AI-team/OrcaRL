# Rover installation:

## Network setup

To set a wired IPv4 address manually on a Jetson NX Xavier, you can use the `nmcli` command-line tool, which is part of the NetworkManager package. This tool allows you to configure network settings, including setting a static IP address for your wired connection. 

1. Open a terminal on your Jetson NX Xavier.
2. Use the `nmcli con` command to modify your wired connection. 
3. Replace "Wired connection  1" with the name of your wired connection, which you can find by listing your network connections.

```bash
sudo nmcli con mod "Wired connection 1" ipv4.addresses "192.168.1.102/24" ipv4.gateway  192.168.1.1 
ipv4.method "manual"
```

3. Apply the changes and restart the network connection:

```bash
sudo nmcli con down "Wired connection 1" && sudo nmcli con up "Wired connection 1"
```

## SSH connection setup

To set up an SSH connection and copy your SSH key to a remote server, follow these steps:

1. **Generate an SSH Key Pair**: If you haven't already, you need to generate an SSH key pair on your local machine. You can do this using the `ssh-keygen` command. By default, this will create a private key (`id_rsa`) and a public key (`id_rsa.pub`) in the `~/.ssh` directory.

```bash
ssh-keygen
```

To copy your SSH public key to a remote server from a Windows machine, you can use the following PowerShell command:

```bash
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh rover@192.168.1.102 "cat >> .ssh/authorized_keys"
```

2. **Copy the Public Key to the Remote Server**: Once you have your SSH key pair, you can use the `ssh-copy-id` command to copy your public key to the remote server. This command will append the public key to the `~/.ssh/authorized_keys` file on the remote server, allowing you to authenticate using your private key.

```bash
ssh-copy-id jetson@192.168.1.8
```

3. **Connect to the Remote Server**: After copying your public key, you can connect to the remote server using SSH without entering your password.

```bash
ssh rover@192.168.1.102
```

If you encounter issues with `ssh-copy-id` not working, it might be because the remote server is configured to only accept public key authentication and you don't already have another key on that machine to connect with. In such cases, you can manually copy your public key to the remote server using `scp` and then append it to the `~/.ssh/authorized_keys` file on the remote server.

```bash
scp ~/.ssh/id_rsa.pub jetson@192.168.1.8:/tmp/id_rsa.pub
ssh jetson@192.168.1.8 'cat /tmp/id_rsa.pub >> ~/.ssh/authorized_keys'
```

## Enable interfaces

To install all CAN (Controller Area Network) and I2C interfaces on a Jetson NX Xavier running Ubuntu 20.04 and ensure they are functional with Python libraries, follow these steps:

### CAN Interface Installation

1. **Enable CAN interface**: First, you need to enable the CAN interface on your Jetson NX. This is typically done by enabling the CAN interface in the BIOS/UEFI settings of your Jetson NX.

2. **Install SocketCAN**: SocketCAN is a set of open-source CAN drivers and a network interface that allows you to communicate with CAN devices using standard socket programming interfaces. Install SocketCAN by running the following command:
   ```bash
   sudo apt-get install iproute2 can-utils
   ```

3. **Load CAN kernel module**: Load the CAN kernel module to make the CAN interface available. The specific module to load depends on the CAN hardware you are using. For example, to load the `can-raw` module, use:
   ```bash
   sudo modprobe can-raw
   ```

4. **Verify CAN interface**: Verify that the CAN interface is available and working by listing the network interfaces:
   ```bash
   ip link show
   ```

### I2C Interface Installation

1. **Enable I2C interface**: Ensure that the I2C interface is enabled on your Jetson NX. This can usually be done through the device tree configuration, but it might already be enabled by default.

2. **Install I2C tools**: Install the I2C tools package to interact with I2C devices:
   ```bash
   sudo apt-get install i2c-tools
   ```

3. **Load I2C kernel module**: Load the I2C kernel module to make the I2C interface available. The specific module to load depends on the I2C hardware you are using. For example, to load the `i2c-dev` module, use:
   ```bash
   sudo modprobe i2c-dev
   ```

4. **Verify I2C interface**: Verify that the I2C interface is available and working by listing the I2C devices:
   ```bash
   i2cdetect -l
   ```

### Python Libraries for CAN and I2C

For Python development, you can use libraries such as `python-can` for CAN and `smbus2` for I2C. Install them using pip:

- For CAN:
 ```bash
 pip install python-can
 ```

- For I2C:
 ```bash
 pip install smbus2
 ```

### Example Usage

- **CAN Example**:
 ```python
 import can

 def receive_can_messages(channel):
      bus = can.interface.Bus(channel=channel, bustype='socketcan')
      while True:
          message = bus.recv()
          print(message)

 receive_can_messages('can0')
 ```

- **I2C Example**:
 ```python
 from smbus2 import SMBus

 def read_i2c_device(address, register):
      with SMBus(1) as bus: # Use the correct bus number
          data = bus.read_byte_data(address, register)
          print(f"Read data: {data}")

 read_i2c_device(0x20, 0x00) # Example I2C address and register
 ```

Remember to replace placeholders like `can0`, `0x20`, and `0x00` with the actual values for your setup.


Enable CAN interface when booting
```bash
# Install system/systemd/network/80-can.txt
sudo cp ./rootfs_configs/etc/systemd/network/80-can.network /etc/systemd/network

# Install system/modules-load.d/can.conf
sudo cp ./rootfs_configs/etc/modules-load.d/can.conf /etc/modules-load.d/

# Comment out or remove /etc/modprobe.d/blacklist-mttcan.conf
sudo nano /etc/modprobe.d/denylist-mttcan.conf

# Activate systemd-networkd service
sudo systemctl enable systemd-networkd

# Start the service immediately:
sudo systemctl start systemd-networkd

# Verify the status to ensure it's running correctly:
sudo systemctl status systemd-networkd
```

## Copy Gumich rover project

Copy Gumich rover jroject to the repository.

```bash
/home/rover/gr_platform/*
```

## ROS installation

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/ROS1-installation.git
chmod +x ROS1-installation/ROS.sh
sudo ./ROS1-installation/ROS.sh
```

The code below needs only executed once:
```bash
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/rover/gr_platform/ros/devel/setup.bash" >> ~/.bashrc
echo 'export DISPLAY=":0"' >> ~/.bashrc
echo 'export OPENBLAS_CORETYPE="ARMV8"' >> ~/.bashrc
echo "export PATH=$PATH:/usr/local/cuda/bin" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64" >> ~/.bashrc
echo "export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda" >> ~/.bashrc
echo "export ROS_IP=192.168.1.102" >> ~/.bashrc
echo "export ROS_HOSTNAME=192.168.1.102" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.1.102:11311" >> ~/.bashrc
source ~/.bashrc
```

## Python dependencies installation

```bash
sudo /usr/bin/python3 -m pip install -r ~/gr_platform/requirements.txt
```

## PCL installation

To build the Point Cloud Library (PCL) from the GitHub repository on Ubuntu  20.04, follow these steps:

1. Install Dependencies: PCL depends on several libraries. Before building PCL, ensure you have all the necessary dependencies installed. You can install them using the following command:

```Bash
sudo apt-get update
sudo apt-get install build-essential cmake git libusb-1.0-0-dev libusb-dev libudev-dev libpcap-dev libeigen3-dev libflann-dev libboost-all-dev libvtk7-dev libqhull-dev libopenni2-dev
```

2. Clone the PCL Repository: Clone the PCL repository from GitHub to your local machine.

```Bash
git clone https://github.com/PointCloudLibrary/pcl.git
```

3. Create a Build Directory: It's a good practice to create a separate build directory to keep your source directory clean.

```Bash
cd pcl
mkdir build
cd build
```

4. Configure the Build: Use CMake to configure the build. You can specify the options you want to enable or disable during the configuration process. For a standard build, you can simply run:

```Bash
cmake ..
```

This command will configure the build system with default options. If you want to enable or disable specific modules or features, you can do so by adding -D<option>=ON/OFF to the command. For example, to enable the visualization module, you would use:

```Bash
cmake -D BUILD_visualization=ON ..
```

5. Build PCL: Once the configuration is complete, build the library using the make command. This process may take some time depending on your system's performance.

```Bash
make -j$(nproc)
```

The -j$(nproc) option tells make to use all available CPU cores for the build process, which can significantly speed up the build time.

6. Install PCL: After the build process is complete, you can install PCL on your system.

```Bash
sudo make install
```

This will install PCL and its libraries to your system. After installation, you may need to update your LD_LIBRARY_PATH environment variable to include the path to the PCL libraries.

7. Update LD_LIBRARY_PATH:

```Bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
sudo ln -s /usr/local/include/pcl-1.10 /usr/include/pcl-1.10
```

You might want to add this line to your `.bashrc` file to make the change permanent.

After following these steps, you should have PCL successfully built and installed on your Ubuntu  20.04 system.

A similar problem while installing AlmaBTE in ubuntu 20.04

```bash
/home/sy/applications/almabte-v1.3.2/src/superlattice_builder.cpp:38:10: fatal error: boost/uuid/sha1.hpp: No such file or directory
   38 | #include <boost/uuid/sha1.hpp>
      |          ^~~~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [src/CMakeFiles/superlattice_builder.dir/build.make:63: src/CMakeFiles/superlattice_builder.dir/superlattice_builder.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:474: src/CMakeFiles/superlattice_builder.dir/all] Error 2
make: *** [Makefile:95: all] Error 2
```

The boost library was okay, however, I find the file 'sha1.hpp' located in another dir. So I just copied it to the right place and it worked;

```bash
sudo cp /usr/include/boost/uuid/detail/sha1.hpp /usr/include/boost/uuid/
```

## Cuda package installation

```bash
sudo apt install cuda && sudo apt-get install nvidia-cuda
```

## ROS dependencies installation

```bash
rosdep install --from-paths src --ignore-src -y --skip-keys pcl --skip-keys obstacle_avoidance_gencfg

sudo apt-get install ros-noetic-geodesy
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-nmea-msgs
sudo apt-get install libsuitesparse-dev
sudo apt-get install libpcl1 ros-noetic-octomap-* -y
sudo apt-get install ros-noetic-tf2-sensor-msgs
```

## Odrive installation

```bash
sudo apt install -y python3-wrapt
sudo apt install i2c-tools
pip install odrivetool

sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```

## Ardupilot installation

MAVROS is compatible with all recent versions of ROS including Kinetic, Melodic and Noetic.

Instructions for `installing MAVROS can be found here <https://github.com/mavlink/mavros/tree/master/mavros#installation>`__ but in short involve running the following command.

```bash

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```
For ease of use on a desktop computer, please also install RQT

```bash

sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins
```

Enable port for controller
```bash
sudo chmod 777 /dev/ttyACM
```

To set a specific port as `ACM4` for a device with the ID `1209:5741`, you can create a custom udev rule. This rule will match the device based on its vendor and product IDs and create a symbolic link to `/dev/ttyACM*`.

1. **Identify the Device**: First, ensure that the device you want to set as `ACM0` is correctly identified by its vendor and product IDs. You can use the `lsusb` command to list all USB devices and find your device.

2. **Create a Udev Rule**: You'll need to create a new udev rule file in `/etc/udev/rules.d/`. This file will contain the rule that matches your device and creates the symbolic link.

3. **Edit the Udev Rule**: Open a terminal and use a text editor to create or edit the udev rule file. For example, you can use `nano`:

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

4. **Add the Rule**: In the editor, add the following line, replacing `idVendor` and `idProduct` with the actual vendor and product IDs of your device. For your device, the rule would look something like this:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5741", SYMLINK+="ttyACM4"
```

This rule tells udev to create a symbolic link named `ttyACM0` for any device that matches the specified vendor and product IDs.

5. **Reload Udev Rules**: After saving the file and exiting the editor, reload the udev rules to apply the changes:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

6. **Verify the Symbolic Link**: Finally, verify that the symbolic link has been created by listing the contents of the `/dev` directory:

```bash
ls -l /dev/ttyACM*
```

You should see `ttyACM4` listed, indicating that the symbolic link has been successfully created.


## Launch file on the boot

Launch project on ROS can automatically execute on the boot using the `robot_upstart` package

```bash
sudo apt-get install ros-noetic-robot-upstart
```

Install the manual control launch file

```bash
python3 gr_platform/doc/srv_up.py
```

Before running the `systemctl` commands we need to modify the service

```bash
sudo nano /lib/systemd/system/ros-manual-control.service
```
Modify the file to this:

```bash
[Unit]
Description="bringup ros-manual-control"
After=network.target

[Service]
Type=simple
Environment="HOME=/home/rover"
Environment="XDG_RUNTIME_DIR=/home/rover"
Environment="XAUTHORITY=/home/rover/.Xauthority"
ExecStart=/usr/sbin/ros-manual-control-start

[Install]
WantedBy=multi-user.target
```

In the `ros-manual-control-start`, before the `JOB_FOLDER` add this line of script:
```bash
sudo nano /usr/sbin/ros-manual-control-start
```
```bash
log info "ros-manual-control: Using workspace setup file /home/rover/gr_platform/root_configs/scripts/env.sh"
source /home/rover/gr_platform/root_configs/scripts/env.sh
JOB_FOLDER=/etc/ros/noetic/ros-manual-control.d
```

To disable the `ros-manual-control`

```bash
sudo systemctl stop ros-manual-control
sudo systemctl disable ros-manual-control
```

To uninstall ros-manual-control

```bash
python3 gr_platform/doc/srv_down.py
```



## Install checkers

```bash
sudo -H pip install jetson-stats
jtop
```


## AI jetpack (optional)

```bash
sudo apt install nvidia-jetpack
```


## Install OpenCV with cuda

git config --global http.postBuffer 524288000 # Set a larger buffer size
git config --global core.compression 0         # Disable compression

cd ~/gr_platform/build_opencv

Go to jtop and make sure that the configuration file met the requirement version

./build_opencv.sh 4.5.4

python3 demo.py -b=5 -t=7