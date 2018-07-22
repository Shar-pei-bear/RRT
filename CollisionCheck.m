function out = CollisionCheck(x,obstacles)
% Determine if the dot is in the cylinder obstacle
distances = sum((x-obstacles(:,1:2)).^2,2);
if any(distances <= (obstacles(:,3).^2/4))
    out = true;
else
    out = false;
end
end