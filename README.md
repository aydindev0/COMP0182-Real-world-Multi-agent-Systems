# COMP0182-Real-world-Multi-agent-Systems

# create conda environment


conda env create -f envros2.yaml

colcon build --symlink-install

source install/setup.bash

export ROS_LOCALHOST_ONLY=0

export ROS_DOMAIN_ID=30

ros2 topic list

ros2 topic echo <name of rigid body>