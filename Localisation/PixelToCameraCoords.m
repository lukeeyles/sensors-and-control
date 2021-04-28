% Description : Converts pixel coords to coords in camera frame
% Parameters :  pixelindex - [row, col] of pixel in image
%               depth - depth at same location from depth image
%               focallength - camera focal length in mm
%               principlepoint - [px, py] location of camera principle point
%               pixelsize - [width, height] size of pixel in mm
% Return:       localcoords - 3D coords of point in camera frame
function cameracoords = PixelToCameraCoords(pixelindex, depth, focallength, principlepoint, pixelsize)
pixelcoords = [pixelindex(2); pixelindex(1); 1];
K = [focallength/pixelsize(1) 0 principlepoint(1);
    0 focallength/pixelsize(2) principlepoint(2);
    0 0 1];
pixelhomog = pixelcoords*depth;
cameracoords = inv(K)*pixelhomog;