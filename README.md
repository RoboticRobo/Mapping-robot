# Mapping-robot
assignment 3 of robotic course

# How to run on robot
	1. build
	2. copy .exe from /build/release and move to robot through teamviewer
	3. run .exe
	
# Note kinect

(MAT).rows = 480 

(MAT).cols = 640

(MAT).at<uchar>(i,j) 

	- j = rows [0 - 480]
	
	- i = cols [0 - 640]

unit of depth is millimeter

nearest depth -> 500 

# Note server localization

WIFI
	- ssid: ASUS-ISL2 
	- pwd:	asusisl2

URL server
	- 192.168.1.59:8080

Start camera
	- 192.168.1.59:8080/camera/start
	- response
		+ Camera is already running
		
Localization
	-  192.168.1.59:8080/pose

	
Snapshop
	- 192.168.1.59:8080
	
Stop camera
	- 192.168.1.59:8080/camera/stop
 
Example-client 
	- https://github.com/dhbaird/easywsclient/blob/master/example-client.cpp
 
 
right bottom
 id: 8	pos: 236.017, 163.929, 397.266	angle: 1.85717
right top
 id: 8	pos: 240.121, -164.212, 396.376	angle: -19.0336  
left top
 id: 8	pos: -211.61, -167.474, 406.818	angle: 4.2893
left bottom
 id: 8	pos: -190.826, 168.746, 408.63	angle: -1.11681
 
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055
id: 8   pos: 209.104, 164.246, 410.03   angle: -82.2055