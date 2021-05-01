%% Rotate2Theta v1
clear;
close;
clc;

%% Connect to TurtleBot

% initializes ROS and connect to the TurtleBot
rosinit;
% display all available ROS topics
rostopic list;

%% Create publisher for Twist messages to control velocity

% create a publisher for the /cmd_vel topic 
robotCmd = rospublisher("/cmd_vel");
% create a message determined by the topic published by the publisher
robotTwist = rosmessage(robotCmd);

%% Get initial Odometry

% Create a subscriber for the odometry messages
odomSub = rossubscriber("/odom");           
odomMsg = receive(odomSub);        % Wait for the subscriber to return data. Has a timeout value of 3 seconds, if nothing received in 3 seconds --> then error

% Extract the current theta from odometry topic
pose = odomMsg.Pose.Pose;           % need to acces the pose message which is 2 levels into nav_msgs/Odometry structure
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
currentTheta = rad2deg(angles(1))         % quat2eul returns euler angle rotations in order of "ZYX" --> Only interested in Z-axis (first element)


%% Rotate to goal orientation

% Set desired Orientation from odometry
goalTheta = 175;
% Set allowable error
goalThetaThreshold = 0.1;

% Keep rotating and comparing the current theta while it is not within the range of the desired theta
while~((goalTheta-goalThetaThreshold <= currentTheta) && (currentTheta <= goalTheta+goalThetaThreshold))     
    
    % Extract theta
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1));
    [currentTheta, goalTheta]          % Display both thetas
    
    % Set angular velocity
    velAngular = 0.01*abs(goalTheta - currentTheta);        %proportional controller
    
    % Rotation direction decision
    if( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta < currentTheta) )
        robotTwist.Angular.Z = -velAngular;
    elseif( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta > currentTheta) )
        robotTwist.Angular.Z = velAngular;
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta < currentTheta) )
        robotTwist.Angular.Z = -velAngular;
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta > currentTheta) )
        robotTwist.Angular.Z = velAngular;
    else
        robotTwist.Angular.Z = velAngular;
    end
    
    % Publish twist and rotate
    send(robotCmd,robotTwist);

end

% Stop the robot afterwards
robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);


%% Disconnect from Robot

clear
rosshutdown
