export ROS_IP="$(/bin/hostname -I | awk '{print $1}')"
export ROS_MASTER_URI=http://"$(/bin/hostname -I | awk '{print $1}')":11311/
