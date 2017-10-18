# Hexapod Simulation

## Project description
The main aim of this project was to create a simulation of hexapod, based on the real  model. Simulation created under the constraints of actual servos mechanics and geometry. Virtual model works under physical constraints implemented as external forces and have collision detection. Elements of the model connected by the joints which represents a servos in the model.
<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31745970-50ee1440-b465-11e7-87bd-f8e1df59115c.png" width=50%>
</p>

## Instruments
Instruments used for implementation of this project include Bullet Physics Library SDK, C++ programming language and Microsoft Visual Studio/Apple XCode IDEs.

## Creating the model
To create a virtual model, we use simple rigid bodies which were provided by the Bullet SDK. For the main body we use object of class “btBoxShape” which represents a simple box, oriented in global space. Every leg of the model consists of three capsule shaped rigid bodies, oriented along “Y”-axis.

## Setting up elements
Every object created independently of others and needs to be placed correctly in accordance to position in the model. For the legs, we need to set initial position in accordance to position and parameters of the main body. Using origin vector, we set the initial position of the elements and for every next element we need to consider position and length of previous. For the setting origin and rotating rigid bodies, we use bullet class “btTransform”.
<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31746077-e4ef43d0-b465-11e7-8a45-b18cc06a44a2.png" width=60%>
</p>

## Joint structure
We connect elements together using joints as analogs of actual servos of the real prototype. Every joint can move along one axis of rotation.
Using pivot vectors, we can set the position of the joint, but we should consider the orientation of local coordinate axis for both elements we want to connect. Pivot vectors start from the centre of the element and should be directed opposite to each other.
<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31746118-1c7c2e4e-b466-11e7-9de3-db089a7a61cf.png" height="300">
<img src="https://user-images.githubusercontent.com/12521579/31746151-3d71de0a-b466-11e7-8f51-11ca34a94eae.png" height="300">
</p>

As every element in the model has its own local coordinate system, for connecting two elements we need to consider orientation of these systems of coordinates. Limit angles are used to establish the amplitude of movement and level of freedom for the joint, they need to be set in accordance to position of both connected elements.

## Movement
For movement implementation, we created gait patterns for each type of movement. On the picture below depicted the main pattern of gait of hexapod which is used for forward movement, backward movement, turning left and turning right.

<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31747112-425174f8-b46b-11e7-9c7d-553c3147f537.png" height="130">
</p>

Moving the hexapod, we operate with joints, moving them from low limit to up limit and contra versa. As “movement pattern” we use tripod, which means that we are moving 3 legs in single moment of time.

<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31747079-1395896a-b46b-11e7-81d8-b5cebf777ba0.png" height="400">
</p>

Each stage of movement, which is stores in Stage object consists of 3 hinges, up and low limits for each hinge and direction. Movements schemes such as forward movement, backward movement, turning left and turning right we store in arrays of stage objects.
Each type of gait we implemented has the similar pattern, the difference is only of limits, but this implementation of gait is plenty flexible and could be used as framework for further development and adding other types of gait, for example, dancing.

## Controls & States
Hexapod could be examined as Finite-state machine. It has 7 different states. In each moment of time it is in particular state and user can change it’s state by pressing buttons on keyboard. The current state is changes according this State diagram. 
Hexapod start from the state of “Rising up” after which automatically comes to state “Standing by”. To store the current stage we created enumerator, which could be encapsulated by one of these 7 names of states. 

<p align="center">
<img src="https://user-images.githubusercontent.com/12521579/31747180-aaf0bd70-b46b-11e7-89a2-af394e1208ee.png" height="500">
</p>


## About Bullet Physics SDK and how to run the program

This is the official C++ source code repository of the Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects, robotics, machine learning etc.

New in Bullet 2.85: pybullet Python bindings, improved support for robotics and VR

The Bullet 2 API will stay default and up-to-date while slowly moving to a new API.
The steps towards a new API is in a nutshell:

1. The old Bullet2 demos are being merged into the examples/ExampleBrowser
2. A new physics-engine agnostic C-API is created, see examples/SharedMemory/PhysicsClientC_API.h
3. Python bindings in pybullet are on top of this C-API, see examples/pybullet
4. A Virtual Reality sandbox using openvr for HTC Vive and Oculus Rift is available
5. The OpenCL examples in the ExampleBrowser can be enabled using --enable_experimental_opencl

You can still use svn or svn externals using the github git repository: use svn co https://github.com/bulletphysics/bullet3/trunk

## Requirements for Bullet 2

A C++ compiler for C++ 2003. The library is tested on Windows, Linux, Mac OSX, iOS, Android,
but should likely work on any platform with C++ compiler. 
Some optional demos require OpenGL 2 or OpenGL 3, there are some non-graphical demos and unit tests too.

## Contributors and Coding Style information

https://docs.google.com/document/d/1u9vyzPtrVoVhYqQOGNWUgjRbfwfCdIts_NzmvgiJ144/edit

## Requirements for experimental OpenCL GPGPU support

The entire collision detection and rigid body dynamics can be executed on the GPU.

A high-end desktop GPU, such as an AMD Radeon 7970 or NVIDIA GTX 680 or better.
We succesfully tested the software under Windows, Linux and Mac OSX.
The software currently doesn't work on OpenCL CPU devices. It might run
on a laptop GPU but performance will not likely be very good. Note that
often an OpenCL drivers fails to compile a kernel. Some unit tests exist to
track down the issue, but more work is required to cover all OpenCL kernels.

## License

All source code files are licensed under the permissive zlib license
(http://opensource.org/licenses/Zlib) unless marked differently in a particular folder/file.

## Build instructions for Bullet using premake. You can also use cmake instead.

**Windows**

Click on build_visual_studio_vr_pybullet_double.bat and open build3/vs2010/0MySolution.sln
When asked, convert the projects to a newer version of Visual Studio.
If you installed Python in the C:\ root directory, the batch file should find it automatically.
Otherwise, edit this batch file to choose where Python include/lib directories are located.

**Windows Virtual Reality sandbox for HTC Vive and Oculus Rift**

Build and run the App_SharedMemoryPhysics_VR project, preferably in Release/optimized build.
You can connect from Python pybullet to the sandbox using:

```
import pybullet as p
p.connect(p.SHARED_MEMORY) #or (p.TCP, "localhost", 6667) or (p.UDP, "192.168.86.10",1234)
```

**Linux and Mac OSX gnu make**

Make sure cmake is installed (sudo apt-get install cmake, brew install cmake, or https://cmake.org)

In a terminal type:

	./build_cmake_pybullet_double.sh

This script will invoke cmake and build in the build_cmake directory. You can find pybullet in Bullet/examples/pybullet.
The BulletExampleBrowser binary will be in Bullet/examples/ExampleBrowser.

You can also build Bullet using premake. There are premake executables in the build3 folder.
Depending on your system (Linux 32bit, 64bit or Mac OSX) use one of the following lines
Using premake:
```
	cd build3
	./premake4_linux gmake --double
	./premake4_linux64 gmake --double
	./premake4_osx gmake --double --enable_pybullet
```
Then
```
	cd gmake
	make
```

Note that on Linux, you need to use cmake to build pybullet, since the compiler has issues of mixing shared and static libraries.

**Mac OSX Xcode**
	
Click on build3/xcode4.command or in a terminal window execute
	
	./premake_osx xcode4

## Usage

The App_ExampleBrowser executables will be located in the bin folder.
You can just run it though a terminal/command prompt, or by clicking it.


```
[--start_demo_name="Demo Name"]     Start with a selected demo  
[--mp4=moviename.mp4]               Create a mp4 movie of the window, requires ffmpeg installed
[--mouse_move_multiplier=0.400000]  Set the mouse move sensitivity
[--mouse_wheel_multiplier=0.01]     Set the mouse wheel sensitivity
[--background_color_red= 0.9]       Set the red component for background color. Same for green and blue
[--fixed_timestep= 0.0]             Use either a real-time delta time (0.0) or a fixed step size (0.016666)
```

You can use mouse picking to grab objects. When holding the ALT or CONTROL key, you have Maya style camera mouse controls.
Press F1 to create a series of screenshots. Hit ESCAPE to exit the demo app.

Check out the docs folder and the Bullet physics forums for further information.
