% Description : Find goal poses given the square pose and odometry
% Parameters :  squarecentre - location of square [x,y]
%               squarenormal - normal of the square [x,y]
%               odom - robot odometry message (nav_msgs/Odometry)
% Return:       goal1 - first goal facing perpendicular to normal [x,y,theta]
%               goal2 - second goal facing parallel to normal [x,y,theta]
function [goal1, goal2] = FindGoal(squarecentre, squarenormal, odom, offset)
% goal 1 at the perpendicular intersection of robot and square normal
% solve linear eqns to find perpendicular intersection
b = [squarecentre(1) - odom.Pose.Pose.Position.X;
    squarecentre(2) - odom.Pose.Pose.Position.Y];
A = [-squarenormal(2) -squarenormal(1);
    squarenormal(1) -squarenormal(2)];
X = linsolve(A,b);
intersection = squarecentre + X(2)*squarenormal;
diff = intersection - [odom.Pose.Pose.Position.X odom.Pose.Pose.Position.Y];
direction1 = atan2(diff(2),diff(1));
goal1 = [intersection, direction1];

% goal 2 at the square - project out 20cm
direction2 = cart2pol(-squarenormal(1),-squarenormal(2));
if X(2) > 0 % normal faces robot
    goal2 = [squarecentre+offset*squarenormal, direction2];
else % normal faces away from robot
    goal2 = [squarecentre-offset*squarenormal, direction2];
end