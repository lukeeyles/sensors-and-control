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
close;
rgbsub = rossubscriber('/camera/color/image_raw');
depthsub = rossubscriber('/camera/depth/image_raw');
odomsub = rossubscriber('/odom',@OdomCallback);
velpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
global odomGlobal; % to store most recent odometry
global twistmsg;    % to store most recent velocities

% define starting twist message to send
twistmsg = rosmessage('geometry_msgs/Twist');

K = [619.759886932052 0 312.973868633392;
    0 623.938256868104 239.459452920097;
    0 0 1];

% create timer to send twist messages
t = timer;
t.ExecutionMode = 'fixedRate';
t.TimerFcn = 'send(velpub,twistmsg)';
t.Period = 0.1;
start(t);

% main loop
%markernames = ["marker_1.jpg","marker_2.jpg","marker_3.jpg"];
markernames = ["marker_1.jpg","marker_2.jpg"];
DriveToGoal([0,0,0],odomsub,0.02);

pause(1);
initialodom = receive(odomsub,2);

figure(2);
plot(initialodom.Pose.Pose.Position.X,initialodom.Pose.Pose.Position.Y,'r.');
hold on;

for i = 1:numel(markernames)
    startodom = receive(odomsub,2);
    startangle = startodom.Pose.Pose.Orientation;
    startangle = quat2eul([startangle.W,startangle.X,startangle.Y,startangle.Z]);
    startangle = startangle(1);
    
    qrloc = [];
    depth = [];
    rgb = [];
    
    % look for marker
    angleIncrement = pi/12;
    angles = wrapToPi((0:angleIncrement:2*pi)-startangle);
    for angle = angles
        fprintf("Driving to angle: %d\n",rad2deg(angle));
        DriveToAngle(angle,odomsub,0.02);
        pause(0.5);
        
        % read depth and rgb images, convert to matlab format
        depthmsg = receive(depthsub,5);
        rgbmsg = receive(rgbsub,5);
        depth = readImage(depthmsg);
        rgb = readImage(rgbmsg);
        disp("Received images");
        fn = sprintf('images_%f_%f.mat',i,angle);
        save(fn,'rgb','depth');
        
        % look for marker in rgb image
        [pointr,pointl] = MarkerDetection(markernames(i),rgb); % find location of 2 points on marker
        if (~all(pointr==0) && ~all(pointl==0))
            fprintf("Found marker at: %f, %f\n",pointr(1),pointr(2));
            break
        end
    end
    
    if (~all(pointr==0) && ~all(pointl==0))
        % find global 3D coords of card
        globalCoords = FindGlobalCoords([pointr;pointl],rgb,depth,odomGlobal,K);
        fprintf("Marker at: %f, %f\n",globalCoords(1),globalCoords(2));
        figure(2)
        plot(startodom.Pose.Pose.Position.X,startodom.Pose.Pose.Position.Y,'k*');
        plot(globalCoords(1,1),globalCoords(1,2),'r*');
        plot(globalCoords(2,1),globalCoords(2,2),'r*');

        % find normal and centre of card
        [normal,centre] = FindSquarePose(globalCoords(:,1:2));

        % find robot goal poses
        [goal1,goal2] = FindGoal(centre,normal,odomGlobal,0.2)
        
        figure(2)
        plot(goal1(1),goal1(2),'b*');
        plot(goal2(1),goal2(2),'g*');

        % go to goal poses
        DriveToGoal(goal1,odomsub,0.03);
        disp("Reached goal 1");
        DriveToGoal(goal2,odomsub,0.03);
        disp("Reached goal 2");

        % go back to initial position
        goal3 = [initialodom.Pose.Pose.Position.X,initialodom.Pose.Pose.Position.Y,0];
        DriveToGoal(goal3,odomsub,0.02);
    end
end


pause(1)
stop(t); % stop timer
delete(t);
rosshutdown; % shut down ros

function OdomCallback(src,msg)
global odomGlobal;
odomGlobal = msg;
end