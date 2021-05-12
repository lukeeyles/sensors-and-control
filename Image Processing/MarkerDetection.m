
function [pointRight, pointLeft] = MarkerDetection(marker,image)

%% Load reference image, and compute surf features
marker = imread(marker);
ref_img_1_gray = rgb2gray(marker);
ref_pts = detectSURFFeatures(ref_img_1_gray);
[ref_features,  ref_validPts] = extractFeatures(ref_img_1_gray,  ref_pts);
%% visual of markers features
% figure; imshow(marker);
% hold on; plot(ref_pts.selectStrongest(50)); %+++++++++++++++++++
%% Visual of 25 SURF features
% figure;
% subplot(5,5,3); title('First 25 Features');
% for i=1:25                 %++++++++++++++++++++++++++
%     scale = ref_pts(i).Scale;
%     image = imcrop(marker,[ref_pts(i).Location-10*scale 20*scale 20*scale]);
%     subplot(5,5,i);
%     imshow(image);
%     hold on;
%     rectangle('Position',[5*scale 5*scale 10*scale 10*scale],'Curvature',1,'EdgeColor','g');
% end
%% Compare to scene
I = rgb2gray(image);

% Detect features
I_pts = detectSURFFeatures(I);
[I_features, I_validPts] = extractFeatures(I, I_pts);
%% visual of image features
%  figure;imshow(image);
%  hold on; plot(I_pts.selectStrongest(50));   %   +++++++++++++++++++++

%% Compare card image to scene

index_pairs = matchFeatures(ref_features, I_features);

ref_matched_pts = ref_validPts(index_pairs(:,1)).Location;
I_matched_pts = I_validPts(index_pairs(:,2)).Location;
%% visual of matched fetures 
% figure, showMatchedFeatures(image, marker, I_matched_pts, ref_matched_pts, 'montage');
% title('Showing all matches');

%% Define Geometric Transformation Objects
[tform_matrix, ref_inlier_pts, I_inlier_pts] = estimateGeometricTransform(ref_matched_pts, I_matched_pts,'affine', 'MaxNumTrials', 10000, 'MaxDistance', 8, 'Confidence', 60);

%% Draw lines to matched points
%  figure;showMatchedFeatures(image, marker, I_inlier_pts, ref_inlier_pts, 'montage');
%  title('Showing match only with Inliers');

%% Transform corner points 

boxPolygon = [1, 1; % top-left
              size(marker, 2), 1; % top-right
              size(marker, 2), size(marker, 1); % bottom-right
              1, size(marker, 1); % bottom-left
              1, 1]; % top-left again to close the polygon


newBoxPolygon = transformPointsForward(tform_matrix, boxPolygon);

% figure;
% imshow(image);
% hold on;
% line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'G', 'LineWidth', 3);
% title('Detected Box');


%% order all corners  into the order top-left bottom-left top-right bottom roght 
b=newBoxPolygon;
b(5,:) = [];
b = sortrows(b);
if b(2, 2) < b(1,2)
b([1 2],:)=b([2 1],:);
end

if b(4, 2) < b(3, 2)
b([3 4],:)=b([4 3],:);
end

%% Visual of all 4 corners 
% imshow(image);
% axis on 
% hold on;
% line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'y');
% 
% plot(b(1, 1), b(1, 2), 'r+', 'MarkerSize', 10, 'LineWidth', 1);
% plot(b(2, 1),b(2, 2), 'r+', 'MarkerSize', 10, 'LineWidth', 2);
% plot(b(3, 1), b(3, 2), 'r+', 'MarkerSize', 10, 'LineWidth', 3);
% plot(b(4, 1),b(4, 2), 'r+', 'MarkerSize', 10, 'LineWidth', 4);


%% Find two final points for debth detection 
imshow(image);
axis on 
hold on;
line(newBoxPolygon(:, 1), newBoxPolygon(:, 2), 'Color', 'r', 'LineWidth', 2);



% Plot cross at row 100, column 50
% x_L= abs(b(1, 1) + b(2, 1))/2 + abs(b(1, 1) - b(2, 1));
% y_L= abs(b(1, 2) + b(2, 2))/2;
% 
% x_R= abs(b(3, 1) + b(4, 1))/2 - abs(b(3, 1) - b(4, 1))
% y_R= abs(b(4, 2) + b(3, 2))/2;

% Plot cross at row 100, column 50
if b(1, 1) > b(2, 1)
x_L= b(1, 1);
else
x_L= b(2, 1);
end

y_L= abs(b(1, 2) + b(2, 2))/2;


if b(3, 1) < b(4, 1)
x_R= b(3, 1);
else
x_R= b(4, 1);
end

y_R= abs(b(4, 2) + b(3, 2))/2;

plot(x_L, y_L, 'c+', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_R,y_R, 'b+', 'MarkerSize', 10, 'LineWidth', 2);


pointRight=[x_R y_R];
pointLeft= [x_L y_L];
%% if marker not found return 0

if (b(2, 2)-b(1, 2)) - (b(4, 2)-b(3, 2)) > 20 || (b(2, 2)-b(1, 2)) < 20
    pointRight=[0 0]
    pointLeft= [0 0]
end


end

