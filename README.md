# CamBot
<img width="1254" height="1254" alt="CamBot_foto" src="https://github.com/user-attachments/assets/c715b939-fa6e-4c64-a7ec-17aea9ac3535" />

CamBot is an autonomous robot powered by computer vision using the OpenMV camera. The system processes visual input in real time — detecting objects, tracking targets, and making navigation decisions onboard without relying on external computation. Built with OpenMV MicroPython, the project explores embedded machine vision as the primary sense.
It's aim is to be a reliable, affordable and easy to build robot, thanks to which students and people interested in robotics can easily learn about machine vision and autonomous systems.

*WHY CAMBOT?*

CamBot's compact pocket-size design makes it the perfect companion for learning on the go. Whether at home, in the classroom, or on the move, the robot is always ready to turn ideas into hands-on experiments with computer vision and autonomous robotics.
The basic version of the robot is fully 3D printable and uses less components as possible, so you can build it in less than 10 minutes with few tools.In any case, its modular structure makes it easy to customize the robot, allowing you to adapt parts and add components as you wish. For example, if I want the robot to have a mechanical arm, I simply need to connect a servomotor to the designated mount.

<img width="502" height="374" alt="ChatGPT Image Aug 19, 2026, 02_53_29 PM" src="https://github.com/user-attachments/assets/6fb63d49-850f-4082-be39-d8cf8a8bb158" />


_*KEY FEATURES:*_

-OpenMV-powered machine vision <br>
-Pocket-sized design <br>
-3D-printable chassis <br>
-2 motors locomotion <br>
-Real-time visual debugging through OpenMV IDE <br>
-MicroPython <br>
-Modular/expandable architecture <br>
-Fully open-source <br>


*WHAT CAN CAMBOT DO?*

CamBot can be easily programmed to do different tasks. Now you can find basic software for color recognition, visual target following and autonomous exploration (ALPHA).


*EDUCATIONAL PURPOSE:*

-BUILT FOR LEARNING
CamBot is designed as an educational platform rather than a finished consumer robot. Its simple architecture allows users to understand the complete robotics pipeline:

Perception → Processing → Decision → Motion

Students can observe what the camera sees through OpenMV IDE, modify the vision algorithm in MicroPython and immediately observe how those changes affect the physical behaviour of the robot.


*PROJECT STATUS: ALPHA*

CamBot is currently under active development. Hardware, software and documentation may change. The current release is intended fo experimentation, testing and community feedback!



*--------------------------------------------------HOW DO I START?-----------------------------------------------------*

The few, simple steps are:

- go to "docs" folder and download the .stl files (these are the files to 3D-print the robot's chassis).
  
- in the same folder check for the "robot_components_list" file, and buy the electrical components (wires, screws and other minor components not included in the list!).

- in the same folder open the "instructions" file and follow the step by step guide to build your CamBot.

- Download OpenMV IDE.

- Connect your robot to your computer with USB and with OpenMV's IDE open, connect your camera.

- go to "src" folder and download the desired code ".py", open the .py file with OpenMV IDE and let it run.

- Enjoy!

IMPORTANT! All the software runs on the OpenMV Cam, so you should check: www.openmv.io

