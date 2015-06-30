Project Repository for CSE395 - Project 1 - Group3
3D Scanner with Poitn Cloud Library and Kinect (by using OpenNI)
Gebze Institiue of Technology

library dependencies:
	OpenNI == 1.5.4.0
	github.com/avin2/SensorKinect-unstable --> open source Kinect drivers for OpenNI 1.5.4.0
	Point Cloud Library >= 1.6.0
	Qt >= 4.8.0
	cmake >= 2.8
	ccmake >= 2.8 (optional) --> useful for easily configuring cmake project, uses curses library
	cmake-gui >= 2.8 (optional)  --> useful for easily configuring cmake project, provides gui
	make >= 3.82 --> for compiling on Linux Systems
	VS2010 --> for compiling on Windows systems

	* There are some other dependencies for using Point Cloud Library

Compiling on Linux:
	mkdir build && cd build
	cmake ..
	make

	* using a subdir named "build" is encouraged when using cmake

Compiling on Windows:
	* using a subdir named "build" is encouraged when using cmake

	* cmake-gui can be usable on Windows for generating Visual Studio 2010 project
	to compile project

Usage:
	* Distance between kinect and center of step platform must be in range of 80 - 100 cm.

	* Distance between kinect and center of step platform must be given as input in Concatenate dialog

	* Distance between kinect and center of object must be given as Z value in Concatenate dialog

	* Shifts at horizontal axis can be editable by X value in Concatenate dialog

	* Y value at Concatenate dialog affected by veritical position of object in view