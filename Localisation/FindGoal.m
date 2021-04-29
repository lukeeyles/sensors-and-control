% Description : Find goal poses given the square pose and odometry
% Parameters :  squarecentre - location of square [x,y]
%               squarenormal - normal of the square [x,y]
%               odom - robot odometry message (nav_msgs/Odometry)
% Return:       goal1 - first goal [x,y,theta]
%               goal2 - second goal [x,y,theta]
function [goal1, goal2] = FindGoal(squarecentre, squarenormal, odom)
% goal 1 at the perpendicular intersection of robot and square normal
% solve linear eqns to find perpendicular intersection
b = [squarecentre(1) - odom.Pose.Pose.Position.X;
    squarecentre(2) - odom.Pose.Pose.Position.Y];
A = [-squarenormal(2) -squarenormal(1);
    squarenormal(1) -squarenormal(2)];
X = linsolve(A,b);
intersection = squarecentre + X(2)*squarenormal;
direction = cart2pol(-squarenormal(1),-squarenormal(2));
goal1 = [intersection, direction];

% goal 2 at the square - project out 20cm
goal2 = [squarecentre+0.2*squarenormal, direction];
