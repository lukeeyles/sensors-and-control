% sources:
% https://au.mathworks.com/help/ros/ug/exchange-data-with-ros-publishers-and-subscribers.html
% https://au.mathworks.com/matlabcentral/answers/418071-how-to-use-timer-function-to-send-ros-messages

clear; close;
ip = '192.168.174.128';
rosinit(ip,11311)
rostopic list

global pose; % to store most recent pose

% define twist message to send
twistmsg = rosmessage('geometry_msgs/Twist');
twistmsg.Linear.X = 0.1;
twistmsg.Linear.Y = 0;
twistmsg.Angular.Z = 1;

% define subscriber and publisher
odomsub = rossubscriber('/odom',@OdomCallback);
velpub = rospublisher('/cmd_vel','geometry_msgs/Twist');

% create timer to send twist messages
t = timer;
t.ExecutionMode = 'fixedRate';
t.TimerFcn = 'send(velpub,twistmsg)';
t.Period = 0.1;
start(t);

poses = [];
for i = 1:10
    poses = [poses;pose];
    pause(1);
end

twistmsg.Angular.Z = -1;

for i = 1:10
    poses = [poses;pose];
    pause(1);
end

stop(t);
delete(t);
rosshutdown;

plot(poses(:,1),poses(:,2));

function OdomCallback(src,msg)
global pose;
pose = OdometryTo2DPose(msg);
end