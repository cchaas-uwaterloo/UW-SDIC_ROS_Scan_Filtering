# Beam Follower

## Overview

This package was made from a Beam Robotics hack day on May 31, 2019.

This package takes an image stream, detects either Geese or people, and controls the robot to follow the detected object. 

The detector takes all bounding boxes outputted from darknet_ros (https://github.com/leggedrobotics/darknet_ros) and tracks only one of the objects (based on the one that was first detected near the image center). It outputs a bounding box of only that one object it is tracking.

Currently there are two implemented approaches for control:

  1. Controller: takes the bounding box and tries to center it in the image and follows until it fills the image to a given percentage of the width. This has been tested and works.
  2. Lidar Controller: this takes the lidar points, projects them into the image and saves only the points lying in the bounding box. It then segments the points based on Euclidean distance, takes the cluster that has the most amount of points and controls the robot based on the centroid of that cluster. With this method, you can specify a minimum follow distance. This code has NOT been tested.

