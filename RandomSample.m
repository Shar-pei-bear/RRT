function x = RandomSample(obstacles)
% Generate a random freespace configuration for the robot

while true
    x = rand(1,2) - 0.5;
    if (~CollisionCheck(x,obstacles))
        return
    end
end