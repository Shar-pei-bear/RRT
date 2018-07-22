function out = Dist(x1, x2)
% Compute the distance between two sets of coordinates
out = sqrt(sum((x1 - x2).^2,2));
end
