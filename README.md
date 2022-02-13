# Navigation
This repository contains packages used for setting up odometry of the robot, creating map with kartographer 

and configuring move_base

## How to use this package
* Clone this repository

* Build all packages include in the src folder
* Upload the robokrub_bringup project into the Arduino MEGA used for controlling the mobile base
* Launch the robokrub_bringup.launch from robokrub_bringup_test to set up the robot information
* Launch robokrub_karto.launch in robokrub_karto packages for creating a map
* Launch robokrub_navigation in robokrub_navigation packages to launch move_base server


