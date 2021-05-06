% Description : Converts pixel coords to coords in camera frame
% Parameters :  pixelindex - [row, col] of pixel in image
%               depth - depth at same location from depth image
%               calibrationMatrix - camera calibration matrix constructed
%               with MakeCalibrationMatrix
% Return:       localcoords - 3D coords of point in camera frame
function cameracoords = PixelToCameraCoords(pixelindex, depth, calibrationMatrix)
pixelcoords = [pixelindex(2); pixelindex(1); 1];
pixelhomog = pixelcoords*depth;
cameracoords = inv(calibrationMatrix)*pixelhomog;