This is the ROS-node that listens to the information from QTM track manager
and publishes it on a ROS-topics. 

If the network is changed the IP-address needs to be fixed. This is done by going to the file
QualisysDriver.cpp in the folder ~/catkin_ws/mocap_qualisys/src and editing the numbers on line
35 to the IP-adress of the PC running the QTM track manager when connected to the new network. 

To compile the code run the following commands after eachother:
cd ~/catkin_ws
catkin_make

To run the ROS-node run the following commands after eachother:
cd ~/catkin_ws/src/mocap_qualisys
. start_mocap_qualisys.bash


