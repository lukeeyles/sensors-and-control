function globalcoords = TurtlebotCameraToGlobalCoords(cameracoords, odom, cameraheight)
RobotRotation = 2*acos(odom.Pose.Pose.Orientation.W);
ObjectCameraT = transl(cameracoords);
CameraRobotT = transl([0 0 cameraheight])*trotx(pi/2)*troty(pi/2);
RobotGlobalT = transl(odom.Pose.Pose.Position.X, odom.Pose.Pose.Position.Y, 0)*...
    trotz(RobotRotation);
ObjectGlobalT = RobotGlobalT*CameraRobotT*ObjectCameraT;
globalcoords = ObjectGlobalT(1:3,4);