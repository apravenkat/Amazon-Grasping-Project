# Amazon-Grasping-Project
This repository consists the vision, motion planning and tactile sensing scripts for the Amazon Grasping Project
# Vision
This folder contains script that identifies the grasping location of objects. The script uses feature matching algorithm to match the target object and searches the observation space for the target object. Once the enough matching features are found, a bouding box is drawn around the object. Then the coordinates of the object are transformed to world coorinates followed by another transformation which gives the coordinates of the object with respect to the end effector of the UR5e. <br /> 
<br /> 
Before cloning this script into the repo, clone and follow the instructions in `https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy` to ensure that the RealSense wrapper has been correctly installed.
<br />
Now you can clone this script into the src directory of the realsense2_camera project.
<br />

The script takes the target image labelled "1.jpg" and scans the camera feed for the target image which is found using SIFT feature matching. This script also contains a rostopic publisher, with the name "Location" with a publisher with name "LocationPublisher". The data is outputted as a String in the format "x: X.XX, y: Y.YY, z: Z.ZZ", all the numbers are in meters. The outputted numbers are the distances of the object from the end-effector of the robot. <br/>
The image shows the script detecting the object and the publisher outputting the data.
<br/>
![image](https://user-images.githubusercontent.com/92841422/215548890-e7adca09-ac4f-4580-b5c8-a0231a35bb85.png)
<br />
<br />
You can run the script by
<br/>
`rosrun realsense2_camera RecogSIFT.py`
