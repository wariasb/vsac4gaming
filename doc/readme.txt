-----------------------------
To Compile + Run
-----------------------------
cmake -S . -B ./build -DCMAKE_INSTALL_PREFIX=./install
cmake --build ./build
cmake --install ./build
. ./install/share/vsac4gaming/local_setup.bash
./build/vsac_app

---------------------------------------
To check used defined topic
---------------------------------------
source /opt/ros/$ROS_DISTRO/setup.bash
. ./install/share/vsac4gaming/local_setup.bash
ros2 interface show vsac4gaming/msg/topicName
ros2 topic echo /topicName