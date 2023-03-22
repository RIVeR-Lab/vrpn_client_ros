# vrpn_client_ros
This for aims at porting the original code from the kinetic-devel branch to ROS2.

## Requirements - Ubuntu 22.04 and ROS2 Humble
        git clone git@github.com:RIVeR-Lab/vrpn_client_ros.git
        cd vrpn_client_ros
        git checkout humble

        git clone git@github.com:vrpn/vrpn.git
        mkdir build
        cd build
        cmake ..
        sudo make -j4
        sudo make install

        #relative tracking:
        ros2 launch vrpn_client_ros egocentric.launch.py
