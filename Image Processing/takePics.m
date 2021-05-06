clear;
close;
clc;

%% Connect to TurtleBot
rosshutdown
% initializes ROS and connect to the TurtleBot
rosinit('192.168.0.20',11311);
% display all available ROS topics
rostopic list;
%%
rgbsub = rossubscriber('/camera/color/image_raw');
depthsub = rossubscriber('/camera/depth/image_raw'); % not sure if topic name is correct

%%
depthmsg = receive(depthsub,2);
rgbmsg = receive(rgbsub,2);
depth = readImage(depthmsg);
rgb = readImage(rgbmsg);
