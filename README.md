Indoor Absolute Localization using RGB and Depth Sensors
========================================================

Purpose
=======

The purpose of this project is to combine the depth and RGB camera data available in the Microsoft Kinect to triangulate absolute position from known room features.

Scope
=====

Mobile robots operating indoors must be capable of navigating often cluttered environments without access to GNSS data. There are existing techniques that can assist the indoor robot, such as Wifi beacons, however these techniques are ususceptible to modifications to the environment. Optical or range sensors offers an alternative approach that has recently become cost-effective.

The Microsoft Kinect sensor is an inexpensive combination of multiple sensors and PrimeSense System-on-Chip (SoC) sensor fusion that provides both depth and video via USB. Recent publication of experimental data shows that the Kinect is accurate enough for indoor mobile robotics applications[1]. 

In addition to using commercial off-the-shelf hardware, this project will leverage the OpenCV (Open Source Computer Vision) library. OpenCV provides over 500 functions for general image processing, segmentation, transforms, machine learning, geometric descriptors, features, tracking, matrix math, camera calibration, fitting, utilities, and data structures[2].

In this project, a computer vision algorithm that uses the depth and RGB data from the Kinect will be developed to estimate the absolute position using a priori knowledge of indoor landmarks (e.g. walls, lights, doors). The robot will be stationary, will not assume a predetermined starting position, and will not rely on external sensors such as GPS.

This project is unique in that it will take advantage of the Kinect's limited depth range. For example, as it was shown in Huang10 [1] where the depth data is missing, e.g. florescent lights, RGB data is present. The approach developed here will exploit this "limitation" and use it to classify the room's features, identify landmarks, and from this triangulate the camera's position.

Algorithm
=========
<pre><code>

  +-------+    +--------+    +------- +    +--------+    +--------+
  |       |    |        |    |        |    |Straight|    |        |
  |  RGB  |+-->|Landmark|+-->| Noise  |+-->|  Line  |+-->|  Edge  |+------------+
  | Camera|    |Filter  |    | Filter |    |Detector|    |Detector|             |
  +-------+    +--------+    +------- +    +--------+    +--------+             |
                    ^                                                           v
                    |                                                    +--------------+
                    |                                                    |              |
                    |             +-------------------------------------+| Perspective  |
                    |             |                                      |Transformation|
                    |             |                                      +--------------+
                    +             v
                +-------+    +--------+
                |       |    |        |
                | Depth |+-->|Centroid|+------------+
                | Camera|    |        |             |
                +-------+    +--------+             |
                                                    v
                                              +----------+
                                              |          |
                                              |   Pose   |
                                              |Estimation|
                                              +----------+
</code></pre>

The first stage of the algorithm is to combine the depth and RGB data to identify regions where depth data is missing. This will remove objects from the scene that are not windows or lights leaving only those landmarks. Next, to produce the perspective transformation the RGB data must be filtered for noise (e.g. Gaussian), run through an edge detector (e.g. Canny), and finally a straight line filter (e.g. Hough). These combine to produce a black and white image containing only polygons. Next, the centers of these polygon are computed (i.e. centroids) and with the depth data of one or more (ideal) polygons the pose estimation is performed. Finally, this yields the Kinect's position and orientation between the observed 2D projections and their 3D positions in the world frame.


Objectives
==========

 1. Demonstrate a Kinect SDK or Libfreenect test application that displays RGBD (combined camera and range data)
  * Note: no CSUF equipment is needed
 1. Gather experimental data of range data looking at ceiling lights from various angles (e.g. dead center, from bottom left, etc...)
  * Modify test program to extract range data from surrounding areas
  * Create mathematical model given test perspectives and test depths
 1. Experiment with test patterns and materials that can be placed on the landmark to assist in identifying the landmark (e.g. which window) and orientation
 1. Design filters to isolate landmarks of interest
 1. Develop triangulation algorithm that will take landmarks in the Kinect's FOV and known room geometries to estimate absolute position
 
Timeline
========

At least every two weeks there will be a deliverable and progress report due.

 * 2012/09/04 (Tue): Proposal
 * 2012/09/05 (Wed): Demo test application
 * 2012/09/07 (Fri): Revised Plan
 * 2012/09/19 (Wed): Filter Test
 * 2012/10/03 (Wed): Position Test
 * 2012/10/17 (Wed): Field Test
 * 2012/10/31 (Wed): Analysis of Experiments
 * 2012/11/14 (Wed): Draft Report
 * 2012/11/28 (Wed): Final Report
 * 2012/12/05 (Wed): Presentation

Risk
====

The first major risk will be overcoming the default mode maximum operating range of 4 meters (13.1234 feet)[1]. The Kinect IR emitter, receiver, and RGB camera were placed relative to each other and calibrated to provide optimal detection of skeletal bodies. As such, there may be large accuracy issues if the range is pushed. To minimize these effects the project will initially work in a room of average size and stay within operating ranges were depth data around landmark can be distinguished.

The second risk is operating the Kinect in an environment with a lot of infrared. Rooms that are brightly lit from outside light will therefore be avoided until appropriate filters can be developed.

Quality and Success Criteria
============================

Estimated 3D coordinates will be compared against ground truth. A successful project will be software that can be used by future indoor robot navigation projects. 

Algorithm
=========

 1. Acquire RGBD frames of scene
 1. Filter where depth sensor shows no data but where RGB does, keeping the depth data around the "dark" spots
 1. Identify rectangular objects on ceilings and walls
 1. Interpolate center of object
 1. Using adjacent valid depth data around object triangulate camera's position

Related Work
============


Development Environment
=======================

The development system used is an Ubuntu 12.04 64-bit laptop. The following commands document the setup of said workstation. This assumes the GCC toolchain and C/C++ libraries are installed.

 1. `sudo add-apt-repository ppa:floe/libtisch`
 1. `sudo apt-get update`
 1. `sudo apt-get install libfreenect libfreenect-dev libfreenect-demos`
 1. `sudo adduser $USER plugdev`
 1. `sudo adduser $USER video`
 1. `cd /etc/udev/rules.d/`
 1. `sudo wget https://raw.github.com/OpenKinect/libfreenect/master/platform/linux/udev/51-kinect.rules`
 1. `sudo apt-get install libopencv libopencv-dev libopencv-core-dev libcv libcv-dev libcvaux libcvaux-dev libhighgui libhighgui-dev opencv-doc libopencv-ml libopencv-ml-dev`
 1. `sudo apt-get install libusb-1.0-0-dev`

To build the application against libfreenect and OpenCV.

 * `gcc source.c -lfreenect_sync ``pkg-config --cflags --libs opencv``

Experiments
===========


Future Possibilities
====================

The following are potential research topics that can build on the work presented in this project.

 * What starting locations and orientations work best?
 * If the robot moved around could the estimates be improved?
 * How could the process be optimized to minimize startup delay?
 * If the algorithm could be made efficient and fast, should the calculation of absolute position be executed continuously?
 * After position is determined orientation can also be found estimated based of the depth and angle to the landmark corners
 * Can a magnetometer be used to assist in oriented the robot instead of using markers?

References
==========

 1. El-laithy, R.,  Huang, J., & M. Yeh: _Study on the Use of Microsoft Kinect for Robotics Applications, Proceedings of IEEE/ION Position, Location and Navigation Symposium (PLANS) 2012_, Myrtle Beach, SC, Apr. 2012 
 1. _OpenCV Wiki_, 09 Sep 2012, <http://opencv.willowgarage.com/>

