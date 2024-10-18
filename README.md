# Installation and Assembly Guide

## Disclaimer:
The project was created by a team of students under the Adaptive Robotics minor at Fontys University of Applied Sciences. The goal of the project was to create a plug and play introduction to ROS 2 Humble and mobile robotics. By following the guide anyone should be able to create their own holonomic platform and install all the necessary programs in order to create a map of the environment and autonomously navigate within the map.

# Hardware Guide:
### 3D printed parts:
All the 3D models are located inside the &&&&& folder. For 3D printing use the .stl files. STL files, Standard Triangle Library, files are the most commonly used file format for 3D printing. These files has to be sliced by a slicing software before the 3D printer could interpret them. For slicing Cura slicer is recommended with the following print parameters:

| Print parameters:          | Value                                  |
| -------------------------- | -------------------------------------- |
| Material:                  | Generic PLA                            |
| Print temperature:         | 205 C                                  |
| Print speed:               | 50 mm/s or less (aim for good quality) |
| Layer height:              | 0.1 or 0.12 mm                         |
| Wall count:                | 3                                      |
| Bottom and top Wall count: | 8                                      |
| Infill:                    | 20% gyroid                             |
| Build plate adhesion:      | brim (recommended)                     |

For printing Ultimaker and Bambu Lab printers were used, these printers provide high quality however they are not mandatory for the project since the plastic parts are plenty strong by themselves.

For the mechanical assembly there is also a full CAD assembly provided which could be used as reference. 

### Electrical components:
All the necessary components are located in the BOM (bill of material) excel file. There is also a possible vendor provided, however you are free to choose wherever to source the components from. Additionally it is important to mention that the Lidar used for this project, RPLidar, model: A2M8, is currently discontinued. The alternative A2M12 requires changes to the code. These changes are not implemented in the current working repository! The implementation of the A2M12 model is only recommended for experienced ROS users.

### Electrical Schematics:
[Arduino Wiring Schematics](Arduino_Code/README.md)

Hadi insert schematics here as a picture <3 



# Software Guide:

### Raspberry Pi side:
[Ubuntu installation Guide](https://raspberrytips.com/headless-ubuntu-server-installation/)


### Computer side: