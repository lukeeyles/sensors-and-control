function DriveToAngle(angle,odomsub)
    
global twistmsg;

%% Get initial Odometry
  
odomMsg = receive(odomsub);
% Extract the current theta from odometry topic
pose = odomMsg.Pose.Pose;           % need to acces the pose message which is 2 levels into nav_msgs/Odometry structure
quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
currentTheta = rad2deg(angles(1))         % quat2eul returns euler angle rotations in order of "ZYX" --> Only interested in Z-axis (first element)

%% Rotate to goal orientation

% Define goal theta
goalTheta = rad2deg(angle);
% Set allowable error
goalThetaThreshold = 1;

% Keep rotating and comparing the current theta while it is not within the range of the desired theta
while~((goalTheta-goalThetaThreshold <= currentTheta) && (currentTheta <= goalTheta+goalThetaThreshold))     
    
    % Extract theta
    odomMsg = receive(odomsub,3);
    pose = odomMsg.Pose.Pose;
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);  
    currentTheta = rad2deg(angles(1));
    [currentTheta, goalTheta]          % Display both thetas
    
    % Set angular velocity
    velAngular = 0.01*abs(goalTheta - currentTheta);        %proportional controller
    
    % Rotation direction decision
    if( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta < currentTheta) )
        twistmsg.Angular.Z = -velAngular;
    elseif( (goalTheta >= 0) && (goalTheta <= 180) && (goalTheta > currentTheta) )
        twistmsg.Angular.Z = velAngular;
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta < currentTheta) )
        twistmsg.Angular.Z = -velAngular;
    elseif( (goalTheta < 0) && (goalTheta >= -180) && (goalTheta > currentTheta) )
        twistmsg.Angular.Z = velAngular;
    else
        twistmsg.Angular.Z = velAngular;
    end
    
%     % Publish twist and rotate
%     send(velpub,twistmsg);

end

% Stop the robot afterwards
twistmsg.Linear.X = 0;
twistmsg.Angular.Z = 0;
%     send(velpub,twistmsg);

end
