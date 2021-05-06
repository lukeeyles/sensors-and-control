% Description : find 2d normal and centre of square from global coords
function [normal,centre] = FindSquarePose(globalCoords,odom)
% want to point the normal towards the robot
centre = mean(globalCoords(:,1:2));