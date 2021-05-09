% Description : find 2d normal and centre of square from global coords
function [normal,centre] = FindSquarePose(globalCoords)
% want to point the normal towards the robot
centre = mean(globalCoords(:,1:2));
parallel = globalCoords(2,1:2) - globalCoords(1,1:2);
normal = [parallel(2) -parallel(1)]./sqrt(parallel(1)^2 + parallel(2)^2);