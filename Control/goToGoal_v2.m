%% goToGoal v2
% given input [x,y,theta]
% move to intersection point then rotate to desired theta

clear;
close;
clc;

%% Connect to TurtleBot

% initializes ROS and connect to the TurtleBot
rosinit;
% display all available ROS topics
rostopic list;


%% Given Goal (from localisation function)

% goals = [x,y,theta] 
goalX = 0;
goalY = 3;
goalTheta = -90;

goalRobot = [goalX,goalY,goalTheta]


%% Create publisher for Twist messages to control velocity

% create a publisher for the /cmd_vel topic 
robotCmd = rospublisher("/cmd_vel");
% create a message determined by the topic published by the publisher
robotTwist = rosmessage(robotCmd);


%% Get initial Odometry

% Create a subscriber for the odometry messages
odomSub = rossubscriber("/odom");           
odomMsg = receive(odomSub);        % Wait for the subscriber to return data. Has a timeout value of 3 seconds, if nothing received in 3 seconds --> then error

pose = odomMsg.Pose.Pose;          % need to acces the pose message which is 2 levels into nav_msgs/Odometry structure

% Extract cartesian coordinates and assign to variables
currentX = pose.Position.X;
currentY = pose.Position.Y;
% z = pose.Position.Z;
% [x,y,z]       % display coordinates
currentPos = [currentX,currentY]

% Extract the current theta
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
currentTheta = rad2deg(angles(1))         % quat2eul returns euler angle rotations in order of "ZYX" --> Only interested in Z-axis (first element)

%%

angleToGoalThreshold = 5;

distanceThreshold = 0.025;
distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 );


while(distanceRemaining > distanceThreshold)
    
    % Listen to o
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;
    % Extract current position
    currentX = pose.Position.X;
    currentY = pose.Position.Y;
    % Extract current theta
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1))
    
    % Calculate how much rotation is needed to face the goal
    angleToGoal = rad2deg(atan2(goalY-currentY,goalX-currentX))
    angleDifference = angleToGoal - currentTheta;
    % Calculate distance remaining to goal
    distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 );
    
    [angleDifference,distanceRemaining]
    
    velAngular = 0.15;
    velLinear = 0.2 + 0.05*distanceRemaining;
    
    
    if( abs(angleDifference) > angleToGoalThreshold &&  angleToGoal > currentTheta )
        robotTwist.Linear.X = 0;
        robotTwist.Angular.Z = velAngular;         % spin CCW
        if( abs(angleDifference) > 180 )
            robotTwist.Linear.X = 0;
            robotTwist.Angular.Z = -velAngular;
        end
    elseif ( abs(angleDifference) > angleToGoalThreshold &&  angleToGoal < currentTheta )
        robotTwist.Linear.X = 0;
        robotTwist.Angular.Z = -velAngular;          %spin CW
        if( abs(angleDifference) > 180 )
            robotTwist.Linear.X = 0;
            robotTwist.Angular.Z = velAngular;
        end
    else
        robotTwist.Linear.X = velLinear;
        robotTwist.Angular.Z = 0;
    end
    
    %Publish twist message to "/cmd_vel" topic
    send(robotCmd,robotTwist);
    
end

% Stop the robot afterwards
robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);

%% Rotate to goal orientation

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
    velAngularMin = 0.1
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

%% Disconnect from Robot

clear
rosshutdown

