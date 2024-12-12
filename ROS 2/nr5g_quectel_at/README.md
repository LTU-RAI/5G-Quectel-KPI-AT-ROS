# 5G Quectel KPI AT ROS2

 Author: [Emanuele Pagliari](https://github.com/Palia95)

 This ROS2 package gathers the main 5G RF quality indexes from Quectel 5G modems (Tested on `RM520N-GL`) and publish them in the proper ROS2 topic (`quectel_nr5g`).

## Requirements

 The only requirements are Python3 with its related ROS2 dependencies and `pySerial` installed. Also, a Quctel 5G modem properly connected to the host computer is needed with a good quality USB-C 3.0 cable.

## To do

Modify the parser and the publisher to include more data.
