function out = LocalPlanner(x1, x2, obstacles)
% Determine if a line segment intersect with the cylinder obstacles
% ax + by + c = 0;
a = x2(1,2) - x1(1,2);
b = x1(1,1) - x2(1,1);
c = x2(1,1)*x1(1,2) - x1(1,1)*x2(1,2);
distances = abs(a*obstacles(:,1)+b*obstacles(:,2) + c)/sqrt(a^2 + b^2);
flag1 = (distances <= (obstacles(:,3)/2));
vector1 = [x1(1,1),x1(1,2)] - obstacles(:,1:2);
vector2 = [x2(1,1),x2(1,2)] - obstacles(:,1:2);
vector3 = [x1(1,1),x1(1,2)] - [x2(1,1),x2(1,2)]';
flag2 = ((vector1*vector3).*(vector2*vector3) < 0);
flag = and(flag1, flag2);
if any(flag)
    out = false;
else
    out = true;
end
end
