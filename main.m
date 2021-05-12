clear; close;
ip = '192.168.0.20';
rosshutdown;

% intialise ros if it isn't initialised
try
    rosnode list
catch exp
    rosinit(ip,11311,'NodeHost','192.168.0.2')
end

%%
% subscribers and publishers
rgbsub = rossubscriber('/camera/color/image_raw');
depthsub = rossubscriber('/camera/depth/image_raw');
odomsub = rossubscriber('/odom',@OdomCallback);
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
markernames = ["marker_1.jpg","marker_2.jpg","marker_3.jpg"];
markernames = ["marker_1.jpg"];
for i = 1:numel(markernames)
    qrloc = [];
    depth = [];
    rgb = [];
    
    % look for marker
    angleIncrement = pi/8;
    for angle = -pi:angleIncrement:pi
        fprintf("Driving to angle: %d",rad2deg(angle));
        DriveToAngle(angle,odomsub);
        
        % read depth and rgb images, convert to matlab format
        depthmsg = receive(depthsub,2);
        rgbmsg = receive(rgbsub,2);
        depth = readImage(depthmsg);
        rgb = readImage(rgbmsg);
        disp("Received images");
        
        % look for marker in rgb image
        [point1,point2] = MarkerDetection(markernames(i),rgb); % find location of 2 points on marker
        if (~all(point1==0) && ~all(point2==0))
            fprintf("Found marker at: %d, %d",point1(1),point1(2));
            break
        end
    end
    
    % find global 3D coords of card
    globalCoords = FindGlobalCoords([point1;point2],rgb,depth,odomGlobal);
    fprintf("Marker at: %d, %d",globalCoords(1),globalCoords(2));
    
    % find normal and centre of card
    [normal,centre] = FindSquarePose(globalCoords(:,1:2));
    
    % find robot goal poses
    [goal1,goal2] = FindGoal(centre,normal,odomGlobal);
    
    % go to goal poses
    DriveToGoal(goal1,odomsub);
    DriveToGoal(goal2,odomsub);
end

stop(t); % stop timer
delete(t);
rosshutdown; % shut down ros

function OdomCallback(src,msg)
global odomGlobal;
odomGlobal = msg;
end