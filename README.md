# Amazon-Grasping-Project
This repository consists the vision, motion planning and tactile sensing scripts for the Amazon Grasping Project
# Vision
This folder contains script that identifies the grasping location of objects. The script uses feature matching algorithm to match the target object and searches the observation space for the target object. Once the enough matching features are found, a bouding box is drawn around the object. Then the coordinates of the ibject are transformed to world coorinates followed by another transformation which gives the coordinates of the object with respect to the end effector of the UR5e.
