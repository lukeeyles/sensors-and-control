% Description : find global coords from pixel coords, depth image and odom
% Parameters :  cardloc - 2*2 matrix of points on the card [row1 col1; row2 col2]
%               rgbimg - rgb image the card was located in
%               depthimg - depth image at the same time
%               odom - turtlebot odometry
%               calibrationMatrix - camera calibration matrix constructed
%               with MakeCalibrationMatrix
% Return :      globalCoords [x1 y1 z1; x2 y2 z2] of card points
function globalCoords = FindGlobalCoords(cardloc,rgbimg,depthimg,odom,calibrationMatrix)
cameraheight = 0.15;
depthidx = size(depthimg).*cardloc./size(rgbimg); % resize coords so we can find the same point on the depth image
depth = [depthimg(depthidx(1,:)); depthimg(depthidx(2,:))];
% TODO transform depth to actual depth in m....

globalCoords = zeros(2,3);
for i = 1:size(depth,1)
    cameracoords = PixelToCameraCoords(cardloc(i,:), depth(i,:), calibrationMatrix);
    globalCoords(i,:) = TurtlebotCameraToGlobalCoords(cameracoords, odom, cameraheight);
end