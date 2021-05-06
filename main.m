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
global twistmsg;    % to store most recent velocities

% define starting twist message to send
twistmsg = rosmessage('geometry_msgs/Twist');

% create timer to send twist messages
t = timer;
t.ExecutionMode = 'fixedRate';
t.TimerFcn = 'send(velpub,twistmsg)';
t.Period = 0.1;
start(t);

% main loop
numcards = 3; % number of cards to read
while numcards > 0
    qrloc = [];
    depth = [];
    rgb = [];
    
    % look for qr code
    angleIncrement = pi/8;
    for angle = -pi:angleIncrement:pi
        DriveToAngle(angle,odomsub); % TODO
        
        % read depth and rgb images, convert to matlab format
        depthmsg = receive(depthsub,2);
        rgbmsg = receive(rgbsub,2);
        depth = readImage(depthmsg);
        rgb = readImage(rgbmsg);
        
        % look for playing card in rgb image
        cardloc = FindCardLocation(numcards); % TODO find location of 2 points on card
    end
    
    % find global 3D coords of card
    globalCoords = FindGlobalCoords(cardloc,rgb,depth,odomGlobal);  % TODO
    
    % find normal and centre of card
    [normal,centre] = FindSquarePose(globalCoords);  % TODO
    
    % find robot goal poses
    [goal1,goal2] = FindGoal(centre,normal,odomGlobal);
    
    % go to goal poses
    DriveToGoal(goal1,odomsub); % TODO
    DriveToGoal(goal2,odomsub); % TODO
end

stop(t); % stop timer
delete(t);
rosshutdown; % shut down ros

function OdomCallback(src,msg)
global odomGlobal;
odomGlobal = msg;
end