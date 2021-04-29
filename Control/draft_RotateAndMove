%% Move robot to a goal  position and orientation

% Desired Odometry
goalX = 3;
goalY = 3;
goalTheta = 45;

% Distance to the goal
goalDist = sqrt((goalX - x).^2 - (goalY - y).^2)

% Allowed goal errors
goalDistThreshold = 0.01;
goalThetaThreshold = 0.1;

while(abs(goalTheta - theta) > goalThetaThreshold)
    robotTwist.Angular.Z = 0.2;
    send(robotCmd,robotTwist);
end

robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);


while(goalDist > goalDistThreshold)
    robotTwist.Linear.X = 0.5;
end

robotTwist.Linear.X = 0;
robotTwist.Angular.Z = 0;
send(robotCmd,robotTwist);