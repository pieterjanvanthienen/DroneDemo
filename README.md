# Demo on advanced control and smart inspection with drones

![The readme of Mathias and Rian can be found here.](https://github.com/pieterjanvanthienen/DroneDemo/blob/master/rianbeckmathiasbos.md)


[<img width=900 src="https://github.com/pieterjanvanthienen/DroneDemo/blob/master/doc/banner.png"/>](https://www.youtube.com/playlist?list=PLIWGhwVcLILO0_cYFVIMucmvqrzaBkuzw)

Master's thesis project by Pieter-Jan Vanthienen 
**'Demo on advanced control and smart inspection with drones'**  
at KU Leuven, Faculty of Engineering Science, Department of Mechanical Engineering 

## Hardware requirements
* Parrot Bebop 2 drone  
* HTC Vive (HMD, 2 Base Stations, 2 Controllers, Tracker + Dongle)
* Vive Tracker Mount for Parrot Bebop 2 (CAD files available [here](https://github.com/RianBeck/DroneDemo/tree/master/tracker%20mount))
* Gamepad (eg Logitech 710 or Xbox 360 controller) - not strictly necessary but advised for safety reasons

## Software requirements & setup
* Project developed in Ubuntu 16.04
* Python (developed with 2.7)
* ROS (developed with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu))
* [OMG-tools](https://github.com/meco-group/omg-tools)
* [Steam](https://www.linuxbabe.com/desktop-linux/install-steam-ubuntu-16-04-lts-xenial-xerus), SteamVR




## Acknowledgements
This project was built using a number of software packages for which we cannot claim credit. The sources can be found at following links:  
* [OMG-tools](https://github.com/meco-group/omg-tools)  
A Python software toolbox for spline-based optimal motion planning.

* [Bebop Autonomy](https://bebop-autonomy.readthedocs.io/en/latest/)  
A ROS interface for the Parrot Bebop 2 drone. This package provides an interface for decoupled x-, y- (roll and pitch angle) and z- (vertical velocity) input commands to the drone.  

* [Triad OpenVR](https://github.com/TriadSemi/triad_openvr)  
A python interface with the HTC Vive tracking system based on *pyopenvr*. We make the small addition in the use of the 'atan2' function rather than 'atan' for the calculation of Euler angles to enable four-quadrant operation.  
* [HTC Keypress](https://gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46)  
Our keypress detection class is based on the example found in the link.
