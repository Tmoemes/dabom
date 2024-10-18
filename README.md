# Installation and Assembly Guide

## Disclaimer

The project was created by a team of students under the Adaptive Robotics minor at Fontys University of Applied Sciences. The goal of the project was to create a plug-and-play introduction to ROS 2 Humble and mobile robotics. By following the guide, anyone should be able to create their own holonomic platform and install all the necessary programs to create a map of the environment and autonomously navigate within the map.

## Hardware Guide

### 3D Printed Parts

All the 3D models are located inside the `3D_Models` folder. For 3D printing, use the `.stl` files. STL (Standard Triangle Library) files are the most commonly used file format for 3D printing. These files need to be sliced by slicing software before the 3D printer can interpret them. For slicing, Cura slicer is recommended with the following print parameters:

| Print Parameters           | Value                                  |
| -------------------------- | -------------------------------------- |
| Material                   | Generic PLA                            |
| Print Temperature          | 205°C                                  |
| Print Speed                | 50 mm/s or less (aim for good quality) |
| Layer Height               | 0.1 or 0.12 mm                         |
| Wall Count                 | 3                                      |
| Bottom and Top Wall Count  | 8                                      |
| Infill                     | 20% gyroid                             |
| Build Plate Adhesion       | Brim (recommended)                     |

For printing, Ultimaker and Bambu Lab printers were used. These printers provide high quality; however, they are not mandatory for the project since the plastic parts are plenty strong by themselves.

For the mechanical assembly, a full CAD assembly is provided, which can be used as a reference.

### Electrical Components

All the necessary components are listed in the BOM (Bill of Materials) Excel file. A possible vendor is provided, but you are free to source the components from anywhere. It is important to mention that the Lidar used for this project, RPLidar model A2M8, is currently discontinued. The alternative A2M12 requires changes to the code. These changes are not implemented in the current working repository! The implementation of the A2M12 model is only recommended for experienced ROS users.

### Electrical Schematics
![image](https://github.com/user-attachments/assets/04fa2b5f-c790-43d9-9397-3a3dc7761b07)
For the wiring of the power lines it is recommended to use the 0.5mm gauge wires with Farrell connectors on the end to make connections more safe and reliable. 
For details regarding the Arduino check [Arduino Wiring Schematics](Arduino_Code/README.md).
## Software Guide

### Computer Side
On the computer you want to install Ubuntu Desktop 22.04 LTS. For a guide how to install Ubuntu follow the link: [Install Ubuntu Guide](https://itsfoss.com/install-ubuntu/)Additionally it is recommended to download the Raspberry Pi etcher tool and use that instead of Rufus or Balena etcher since that tool will be necessary later for the Raspberry Pi imaging.


**Follow here from the Raspberry Pi**
The first step is to clone the GitHub repository:
```bash
git clone https://github.com/Tmoemes/dabom.git
```
Now that the GitHub is cloned we can show the folder and navigate within it. To install ROS 2 Humble follow the following guide [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) . After which we need to install additional components for the system so run the following command: 
```bash
ls
cd dabom
bash install.sh
```
Once it is done it is recommended to add the following line to the last line of the .bashrc file. 
```bash
ls -a 
nano .bashrc
```
and add the following line to the end of the text:
```bash 
source ~/opt/ros/humble/setup.bash
```
**End Raspberry Pi installation here**


### Raspberry Pi Side
[Ubuntu Installation Guide For Windows](https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/)
Follow the installation manual and install Ubuntu Server 22.04.5 LTS onto an SD card. During the installation make sure that SSH is enabled. Make sure to remember the password! You can do this by overwriting the setting before installation. Further more make sure the Wi-Fi details are set to the address of a Wi-Fi network that both the computer/laptop and the Raspberry Pi will be connected to. For the Wi-Fi network it is possible to use a home router or a mobile hotspot with internet access enabled.  Once the installation is done plug in the SD card into the Raspberry Pi and provide power to the system.

Now connect your Ubuntu device to the same Wi-Fi network. Now it is necessary to figure out the IP address of the Raspberry PI. [How to discover devices on network?](https://www.tomshardware.com/how-to/scan-for-network-devices)
Now connect to the raspberry pi via SSH:
```bash
sudo ssh nameofthepi@ipaddress
```

Example:
```bash
sudo ssh pi@192.168.91.2
```

Now that we are in the Raspberry Pi terminal follow the same instructions as the Computer side. 