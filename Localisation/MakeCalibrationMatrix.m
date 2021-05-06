function K = MakeCalibrationMatrix(focallength, pixelsize, principlepoint)
K = [focallength/pixelsize(1) 0 principlepoint(1);
    0 focallength/pixelsize(2) principlepoint(2);
    0 0 1];