Indoor Localization using RGB and Depth Sensors
===============================================

Purpose
=======

The purpose of this project is to combine the depth and RGB camera data available in the Microsoft Kinect to triangulate absolute position from known room features.

Abstract
========
Submit to: ION Pacific PNT 2013 (https://www.ion.org/meetings/pnt2013/pnt2013cfa.cfm)
Sessions on 1) Challenging Navigation Topics, 2) Image Aided Inertial Navigation, 3) First Responder PNT

Mobile robots operating indoors often must be capable of navigating without access to GNSS data. Determining the indoor location is one of the key functions of mobile robots before they can operate properly. Existing indoor localization techniques exist, such as Wifi beacons and SLAM, however these techniques are susceptible to tampering, environmental noise, or are costly. Alternatively, permanent landmarks such as windows and ceiling lights can be identified through image processing techniques and used to determine location. In this paper, an algorithm that combines visual and range data using the Microsoft Kinect sensor, Libfreenect driver, and OpenCV computer vision software is proposed. The Microsoft Kinect sensor is a revolutionary low-cost device that combines multiple sensors to provide both depth and RGB video data via USB. Experimental results also showed that the depth data is accurate enough for indoor mobile robotics applications in Huang2012. Experiments from Huang2012 showed that for some indoor features where the depth data was missing, e.g. florescent lights, RGB data was present and the depth data. The approach developed here exploits this "limitation" and uses it to reliably identify a room's features and from this triangulate the camera's position. In addition to using commercial off-the-shelf hardware, this algorithm leverages the OpenCV (Open Source Computer Vision) library, which provides over 500 functions for general image processing, transforms, matrix math, and camera calibration to name a few[2]. The proposed algorithm first identifies landmarks by searching for quadrilaterals that exist in color data but absent from the depth data. The perspective transformation of these landmarks and the depth to their vertices are then used to estimate the position of the robot relative to them. Finally, with a priori knowledge of the fixed coordinates for those landmarks (e.g. lights and windows), the robots absolute position with respect to the room frame can be estimated. This paper will present the algorithm as well as preliminary results from experiments conducted in a lab environment.

Innovative
==========

The Microsoft Kinect RGB and depth sensor low-cost and USB interface and the open-source OpenCV library has made is possible for anyone to begin experimenting with computer vision and navigation. This project is unique in that it attempts to exploit an inherent weakness in the devices performance indoors to produce a more robust algorithm. An indoor mobile robot equipped with a Kinect sensor can autonomously perform localization within moments on powering on and without have to move to communicate with external systems.

References
==========

 1. El-laithy, R.,  Huang, J., & M. Yeh: _Study on the Use of Microsoft Kinect for Robotics Applications, Proceedings of IEEE/ION Position, Location and Navigation Symposium (PLANS) 2012_, Myrtle Beach, SC, Apr. 2012 
 1. _OpenCV Wiki_, 09 Sep 2012, <http://opencv.willowgarage.com/>

Algorithm
=========
<pre><code>
  +-------+    +--------+    +------- +    +--------+    +--------+
  |       |    |        |    |        |    |        |    |Straight|
  |  RGB  |+-->|Landmark|+-->| Noise  |+-->|  Edge  |+-->|  Line  |+------------+
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

The first stage of the algorithm is to combine the depth and RGB data to identify overlapping polygons where depth data is missing. This will remove objects from the scene that are not windows or lights leaving only those landmarks. Next, to produce the perspective transformation the RGB data must be filtered for noise (e.g. Gaussian), run through an edge detector (e.g. Canny), and finally a straight line filter (e.g. Hough). These combine to produce a black and white image containing only polygons. Next, the centers of these polygon are computed (i.e. centroids) and with the depth data of one or more (ideal) polygons the pose estimation is performed. Finally, this yields the Kinect's position and orientation between the observed 2D projections and their 3D positions in the world frame.


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
 * 2012/10/19 (Wed): Filter Test
 * 2012/11/03 (Wed): Position Test
 * 2012/11/17 (Wed): Field Test
 * 2012/11/29 (Mon): Analysis of Experiments
 * 2012/11/31 (Wed): Draft Report
 * 2012/11/28 (Wed): Final Report
 * 2012/12/05 (Wed): Presentation

Risk
====

The first major risk will be overcoming the default mode maximum operating range of 4 meters (13.1234 feet)[1]. The Kinect IR emitter, receiver, and RGB camera were placed relative to each other and calibrated to provide optimal detection of skeletal bodies. As such, there may be large accuracy issues if the range is pushed. To minimize these effects the project will initially work in a room of average size and stay within operating ranges were depth data around landmark can be distinguished.

The second risk is operating the Kinect in an environment with a lot of infrared. Rooms that are brightly lit from outside light will therefore be avoided until appropriate filters can be developed.

Quality and Success Criteria
============================

Estimated 3D coordinates will be compared against ground truth. A successful project will be software that can be used by future indoor robot navigation projects. 

Like any camera the Kinect's sensors require calibration to compensate for the affects of lense distortion. This process is called calibration. Some calibration has been performed on the IR sensor however in order to match the RGB and depth data images additional calibration is required.

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

 * <pre><code>gcc source.c -lfreenect_sync ``pkg-config --cflags --libs opencv``</code></pre>

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
 * After calibration of the cameras and using the camera's instrinsic and extrinsic parameters how much more accurate is this algorithm at determining the absolute position?
 * Can known constraints such as the minimum and maximum distances from any wall or ceiling be used to further reduce the domain to be scanned, thereby speeding up the calculations?


