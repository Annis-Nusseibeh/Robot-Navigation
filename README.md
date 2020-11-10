# Robot-Navigation
Repository for a variety of robot navigation algorithms, filters, and ROS packages

To subscribe to BNO055 IMU messages from an arduino/ photon, make sure to run rosserial. For example:
>`rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200`