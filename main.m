clear; close;
ip = '192.168.174.128';

% intialise ros if it isn't initialised
try
    rosnode list
catch exp
    rosinit(ip,11311)
end

% subscribers and publishers
rgbsub = rossubscriber('/camera/rgb/image_raw','DataFormat','struct');
depthsub = rossubscriber('/camera/depth/image','DataFormat','struct'); % not sure if topic name is correct
odomsub = rossubscriber('/odom',@OdomCallback,'DataFormat','struct');
velpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
global odomGlobal; % to store most recent odometry

% define starting twist message to send
twistmsg = rosmessage('geometry_msgs/Twist');

% create timer to send twist messages
t = timer;
t.ExecutionMode = 'fixedRate';
t.TimerFcn = 'send(velpub,twistmsg)';
t.Period = 0.1;
start(t);

% main loop
numqr = 3; % number of qr codes to read
while numqr > 0
    qrloc = [];
    depth = [];
    rgb = [];
    
    % look for qr code
    angleIncrement = pi/8;
    for angle = -pi:angleIncrement:pi
        reachedGoal = false;
        while ~reachedGoal
            % use p control to calculate control
            [twistmsg,reachedGoal] = DriveToAngle(angle,odomGlobal); % TODO
        end
        
        % read depth and rgb images, convert to matlab format
        depthmsg = receive(depthsub,2);
        rgbmsg = receive(rgbsub,2);
        depth = readImage(depthmsg);
        rgb = readImage(rgbmsg);
        
        % look for qr code in rgb image
        region = FindQRRegion(rgb); % TODO find general location
        [msg,~,qrloc] = readBarcode(rgb,region); % scan qr code
        
        % check if we have successfully read the qr code
        if msg ~= ""
            numqr = numqr - 1;
            break;
        end
    end
    
    % find global 3D coords of qr code
    globalCoords = FindGlobalCoords(qrloc,rgb,depth,odomGlobal);  % TODO
    
    % find normal and centre of qr code
    [normal,centre] = FindQRPose(globalCoords);  % TODO
    
    % find robot goal poses
    [goal1,goal2] = FindGoal(centre,normal,odomGlobal);
    
    % go to goal poses
    reachedGoal = false;
    while ~reachedGoal
        [twistmsg,reachedGoal] = DriveToGoal(goal1,odomGlobal); % TODO
    end
    reachedGoal = false;
    while ~reachedGoal
        [twistmsg,reachedGoal] = DriveToGoal(goal2,odomGlobal); % TODO
    end
end

stop(t); % stop timer
delete(t);
rosshutdown; % shut down ros

function OdomCallback(src,msg)
global odomGlobal;
odomGlobal = msg;
end