function [point,idx] = FindClosestPoint(match, points)
point = points(1,:);
idx = 1;

for i = 2:size(points,1)
    if Distance2D(points(i,:),match) < Distance2D(points(idx,:),match)
        point = points(i,:);
        idx = i;
    end
end

end

function d = Distance2D(p1,p2)
d = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
end