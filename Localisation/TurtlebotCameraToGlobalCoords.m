% Description : convert coordinates in camera reference frame to global
%               reference frame
% Parameters :  cameracoords - [x, y, z] coords in camera reference frame
%               odom - robot odometry message
% Return :      globalcoords - [x, y, z] coords in global reference frame
function globalcoords = TurtlebotCameraToGlobalCoords(cameracoords, odom)
cameraheight = 0.1;
%RobotRotation = 2*acos(odom.Pose.Pose.Orientation.W);
RobotRotation = odom.Pose.Pose.Orientation;
RobotRotation = quat2eul([RobotRotation.W,RobotRotation.X,RobotRotation.Y,RobotRotation.Z]);
RobotRotation = RobotRotation(1);

T_OC = transl(cameracoords);
T_CR = transl([0.1 0.02 cameraheight])*trotx(-pi/2)*troty(pi/2);
T_RG = transl(odom.Pose.Pose.Position.X, odom.Pose.Pose.Position.Y, 0)*...
    trotz(RobotRotation);
T_OG = T_RG*T_CR*T_OC;
globalcoords = T_OG(1:3,4)';