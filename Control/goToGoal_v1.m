%% goToGoal v1
% Given the theta perpendicular to the square's normal
% Rotates the robot that theta then move forward to the intersection point.

% NOTE: Finish position and orientation is not accurate --> Needs more control

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

%% Given Goal (from localisation function)

% goals = [x,y,theta] 
goalRobot(1) = 0;
goalRobot(2) = 0;
goalRobot(3) = -135;
goalRobot

%% Rotate to goal orientation

% Desired orientation
goalTheta = goalRobot(3);
% Set allowable error
goalThetaThreshold = 0.05;

% Keep rotating and comparing the current theta while it is not within the range of the desired theta
while ~((goalTheta-goalThetaThreshold <= currentTheta) && (currentTheta <= goalTheta+goalThetaThreshold))     
    
    % Extract theta
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1));
    [currentTheta, goalTheta]          % Display both thetas
    
    % Set angular velocity
    velAngularMin = 0.01
    robotTwist.Linear.X = 0;
    velAngular = 0.01*abs(goalTheta - currentTheta);        % with proportional controller
    
    if (velAngular < velAngularMin)
        velAngular = velAngularMin;
    end
    
    % Rotation direction decision
    if( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta < currentTheta) )
        robotTwist.Angular.Z = -velAngular;         % clockwise
    elseif( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta > currentTheta) )
        robotTwist.Angular.Z = velAngular;          % anti-clockwise
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta < currentTheta) )
        robotTwist.Angular.Z = -velAngular;         % clockwise
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta > currentTheta) )
        robotTwist.Angular.Z = velAngular;          % anti-clockwise
    else
        robotTwist.Angular.Z = velAngular;          % clockwise
    end
    
    % Publish twist and begin rotating
    send(robotCmd,robotTwist);

end

% Stop the robot afterwards
robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);

pause(1)

%% Get start position of robot
% Extract cartesian coordinates and assign to variables
currentX = pose.Position.X;
currentY = pose.Position.Y;
% z = pose.Position.Z;
% [x,y,z]       % display coordinates
currentPos = [currentX,currentY]

%% Move to goal position

goalX = goalRobot(1);
goalY = goalRobot(2);

% Distance to the goal
distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 )

% set allowable distance error
distanceThreshold = 0.01;

% Keep moving and checking the distance remaining while it is more than the distance threshold
while (distanceRemaining > distanceThreshold)
    
    % Extract current position
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;
    currentX = pose.Position.X;
    currentY = pose.Position.Y;
    
    % Calculate remaining distance to goal
    distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 )
    
    % Set linear velocity
    
    velLinearMin = 0.05;
    velLinear = 0.2*distanceRemaining;        % with proportional controller
    
    if (velLinear < velLinearMin)
        velLinear = velLinearMin;
    end
    
    robotTwist.Linear.X = velLinear;
    robotTwist.Angular.Z = 0;
    
    % Publish twist and move forward
    send(robotCmd,robotTwist);
    
end

% Stop the robot afterwards
robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);

%% Disconnect from Robot

clear
rosshutdown
