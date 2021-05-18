% Description : find global coords from pixel coords, depth image and odom
% Parameters :  cardloc - 2*2 matrix of points on the card [pointright; pointleft]
%               rgbimg - rgb image the card was located in
%               depthimg - depth image at the same time
%               odom - turtlebot odometry
%               calibrationMatrix - camera calibration matrix
% Return :      globalCoords [x1 y1 z1; x2 y2 z2] of card points
function globalCoords = FindGlobalCoords(cardloc,rgbimg,depthimg,odom,calibrationMatrix)
cardloc = fliplr(cardloc); % convert from x,y to row,col
depthcrop = [43.5100000000000,8.51000000000000,436.980000000000,351.980000000000]
depthimg = imcrop(depthimg,depthcrop);

depthidx = round(size(depthimg).*cardloc./size(rgbimg,1,2)) % resize coords so we can find the same point on the depth image

figure(3)
imagesc(depthimg);
hold on;
% find a depth that isn't 0
depthr = depthimg(depthidx(1,1),depthidx(1,2));
while depthr == 0
    % decrement column until we find something
    depthidx(1,2) = depthidx(1,2) - 1;
    depthr = depthimg(depthidx(1,1),depthidx(1,2));
    plot(depthidx(1,2),depthidx(1,1),'g*');
end

% find a depth that isn't 0
depthl = depthimg(depthidx(2,1),depthidx(2,2));
while depthl == 0
    % increment column until we find something
    depthidx(2,2) = depthidx(2,2) + 1;
    depthl = depthimg(depthidx(2,1),depthidx(2,2));
    plot(depthidx(2,2),depthidx(2,1),'g*');
end

depth = [depthl; depthr]

plot(depthidx(1,2),depthidx(1,1),'r*');
plot(depthidx(2,2),depthidx(2,1),'r*');
% transform depth to m
depth = double(depth)/1000;

globalCoords = zeros(2,3);
for i = 1:size(depth,1)
    cameracoords = PixelToCameraCoords(cardloc(i,:), depth(i), calibrationMatrix);
    globalCoords(i,:) = TurtlebotCameraToGlobalCoords(cameracoords', odom);
end