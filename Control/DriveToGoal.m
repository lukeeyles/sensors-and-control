function DriveToGoal(goal,odomsub)

global twistmsg;

%% Define goal
goalX = goal(1);
goalY = goal(2);
goalTheta = rad2deg(goal(3));

%% Get initial Odometry
odomMsg = receive(odomsub);
pose = odomMsg.Pose.Pose;         

% Extract cartesian coordinates and assign to variables
currentX = pose.Position.X;
currentY = pose.Position.Y;
% z = pose.Position.Z;
% [x,y,z]       % display coordinates
currentPos = [currentX,currentY];

% Extract the current theta
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
currentTheta = rad2deg(angles(1))         % quat2eul returns euler angle rotations in order of "ZYX" --> Only interested in Z-axis (first element)

%% Move to goal position

angleToGoalThreshold = 5;

distanceThreshold = 0.025;
distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 );


while(distanceRemaining > distanceThreshold)
    
    % Listen to o
    odomMsg = receive(odomsub,3);
    pose = odomMsg.Pose.Pose;   
    % Extract current position
    currentX = pose.Position.X;
    currentY = pose.Position.Y;
    % Extract current theta
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1));
    
    % Calculate how much rotation is needed to face the goal
    angleToGoal = rad2deg(atan2(goalY-currentY,goalX-currentX));
    angleDifference = angleToGoal - currentTheta;
    % Calculate distance remaining to goal
    distanceRemaining = sqrt( (goalX - currentX).^2 + (goalY - currentY).^2 );
    
    [angleDifference,distanceRemaining];
    
    velAngular = 0.15;
    velLinear = 0.2 + 0.05*distanceRemaining;
    
    
    if( abs(angleDifference) > angleToGoalThreshold &&  angleToGoal > currentTheta )
        twistmsg.Linear.X = 0;
        twistmsg.Angular.Z = velAngular;         % spin CCW
        if( abs(angleDifference) > 180 )
            twistmsg.Linear.X = 0;
            twistmsg.Angular.Z = -velAngular;
        end
    elseif ( abs(angleDifference) > angleToGoalThreshold &&  angleToGoal < currentTheta )
        twistmsg.Linear.X = 0;
        twistmsg.Angular.Z = -velAngular;          %spin CW
        if( abs(angleDifference) > 180 )
            twistmsg.Linear.X = 0;
            twistmsg.Angular.Z = velAngular;
        end
    else
        twistmsg.Linear.X = velLinear;
        twistmsg.Angular.Z = 0;
    end
    
%     %Publish twist message to "/cmd_vel" topic
%     send(velpub,twistmsg);
%     

end

% Stop the robot afterwards
twistmsg.Linear.X = 0;
twistmsg.Angular.Z = 0;
% send(velpub,twistmsg);

%% Rotate to goal orientation

% Set allowable error
goalThetaThreshold = 0.05;

% Keep rotating and comparing the current theta while it is not within the range of the desired theta
while ~((goalTheta-goalThetaThreshold <= currentTheta) && (currentTheta <= goalTheta+goalThetaThreshold))     
    
    % Extract theta
    odomMsg = receive(odomsub,3);
    pose = odomMsg.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1));
    [currentTheta, goalTheta];          % Display both thetas
    
    % Set angular velocity
    velAngularMin = 0.1;
    twistmsg.Linear.X = 0;
    velAngular = 0.01*abs(goalTheta - currentTheta);        % with proportional controller
    
    if (velAngular < velAngularMin)
        velAngular = velAngularMin;
    end
    
    % Rotation direction decision
    if( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta < currentTheta) )
        twistmsg.Angular.Z = -velAngular;         % clockwise
    elseif( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta > currentTheta) )
        twistmsg.Angular.Z = velAngular;          % anti-clockwise
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta < currentTheta) )
        twistmsg.Angular.Z = -velAngular;         % clockwise
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta > currentTheta) )
        twistmsg.Angular.Z = velAngular;          % anti-clockwise
    else
        twistmsg.Angular.Z = velAngular;          % clockwise
    end
    
%     % Publish twist and begin rotating
%     send(velpub,twistmsg);

end

% Stop the robot afterwards
twistmsg.Linear.X = 0;
twistmsg.Angular.Z = 0;
% send(velpub,twistmsg);


end

