% Description : find global coords from pixel coords, depth image and odom
% Parameters :  cardloc - 2*2 matrix of points on the card [pointright; pointleft]
%               rgbimg - rgb image the card was located in
%               depthimg - depth image at the same time
%               odom - turtlebot odometry
%               calibrationMatrix - camera calibration matrix
% Return :      globalCoords [x1 y1 z1; x2 y2 z2] of card points
function globalCoords = FindGlobalCoords(cardloc,rgbimg,depthimg,odom,calibrationMatrix)
cardloc = fliplr(cardloc); % convert from x,y to row,col
depthcrop = [40,0.5100,445.9800,359.9800];
depthimg = imcrop(depthimg,depthcrop);

meanpoint = mean(cardloc); % centre in rgb image
leftpointscaled = round(size(depthimg).*cardloc(2,:)./size(rgbimg,1,2));
rightpointscaled = round(size(depthimg).*cardloc(1,:)./size(rgbimg,1,2));
meanpointscaled = round(size(depthimg).*meanpoint./size(rgbimg,1,2));
centredistance = meanpointscaled(2) - leftpointscaled(2);

% match centroid in depth image to centre in rgb image
arearange = [2000 25000];
centroids = FindCentroids(depthimg,arearange);
closestcentroid = FindClosestPoint(meanpointscaled,fliplr(centroids))
depthidx(1,:) = closestcentroid + [0 centredistance];
depthidx(2,:) = closestcentroid - [0 centredistance];
%depthidx = round(size(depthimg).*cardloc./size(rgbimg,1,2)) % resize coords so we can find the same point on the depth image
depthidx = round(depthidx);
depthidx(:,1) = clamp(depthidx(:,1),1,size(depthimg,1));
depthidx(:,2) = clamp(depthidx(:,2),1,size(depthimg,2));
depthidx

figure(3)
imagesc(depthimg);
hold on;
% find a depth that isn't 0
depthr = depthimg(depthidx(1,1),depthidx(1,2));
while depthr == 0
    % decrement column until we find something
    if depthidx(1,2) < 1
        break
    end
    depthidx(1,2) = depthidx(1,2) - 1;
    depthr = depthimg(depthidx(1,1),depthidx(1,2));
    plot(depthidx(1,2),depthidx(1,1),'g*');
end

% find a depth that isn't 0
depthl = depthimg(depthidx(2,1),depthidx(2,2));
while depthl == 0
    % increment column until we find something
    if depthidx(2,2) > size(depthimg,2)
        break
    end
    depthidx(2,2) = depthidx(2,2) + 1;
    depthl = depthimg(depthidx(2,1),depthidx(2,2));
    plot(depthidx(2,2),depthidx(2,1),'g*');
end

depth = [depthr; depthl]

plot(depthidx(1,2),depthidx(1,1),'r*');
plot(depthidx(2,2),depthidx(2,1),'r*');
% transform depth to m
depth = double(depth)/1000;

globalCoords = zeros(2,3);
for i = 1:size(depth,1)
    cameracoords = PixelToCameraCoords(cardloc(i,:), depth(i), calibrationMatrix);
    globalCoords(i,:) = TurtlebotCameraToGlobalCoords(cameracoords', odom);
end