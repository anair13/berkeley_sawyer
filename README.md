# berkeley_sawyer

To run example:
rosrun berkeley_sawyer test_sdk.py

To run data collection:
rosrun berkeley_sawyer random_pushing

Start the kinect-node:
roslaunch kinect2_bridge kinect2_bridge.launch
Start a visualizer for the kinect-data:
rosrun kinect2_viewer kinect2_viewer sd cloud

Enable/Reset Swayer, e.g. after hitting emergency button:
rosrun intera_interface enable_robot.py -e