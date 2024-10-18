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
![image](https://github.com/user-attachments/assets/7b6bb1f8-dd1e-4b89-bf37-34323ceaf625)

[Arduino Wiring Schematics](Arduino_Code/README.md)




For the wiring of the power lines it is recommended to use the 0.5mm gauge wires with Farrell connectors on the end to make connections more safe and reliable. 

Hadi insert schematics here as a picture <3

## Software Guide

### Raspberry Pi Side

[Ubuntu Installation Guide For Windows](https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/)

### Computer Side
