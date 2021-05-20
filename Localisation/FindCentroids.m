function centroid = FindCentroids(img,arearange)
BW = img > 0;
BW = bwareaopen(BW, 100);
BW = ~bwareaopen(~BW, 100);
figure(4);
imshow(BW);

[B,L] = bwboundaries(BW,'noholes');
stats = regionprops(L,'ConvexArea','Centroid');
centroid = cat(1,stats.Centroid);
area = cat(1,stats.ConvexArea)
areamatch = (area > arearange(1) & area < arearange(2));
centroid = centroid(areamatch,:);

if size(centroid,1) == 0
    disp("NO CENTROIDS FOUND");
end