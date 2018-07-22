function searchtree = RRT(RandomSample, Dist, LocalPlanner, max_tree, start_config, dest_config)
% RRT - rapidly exploring random trees: This procedure computes a rapidly 
% exploring random trees of configuration space. It relies on 3 functions
% RandomSample which generates the coordinate vector for a random sample in
% free space. Dist which measures the distance between two
% coordinate vectors and LocalPlanner which decides whether the 
% motion betweentwo samples are collision-free.
%
% Inputs :
%
%   RandomSample : A function that returns a random sample in freespace
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples
%
%   LocalPlanner :  A function that determines whether there is a collision
%        free straight line path between two points in configuration space
%
%   max_tree : The maximum tree size
%
%
% Output :
%   searchtree - a structure the samples, the edges and edge lengths in the
%        tree

% Array of random samples, each column corresponds to the coordinates
% of a point in configuration space.
samples = inf(max_tree, 2);
samples(1,:) = start_config;

% edges - an array with 2 rows each column has two integer entries
% (i, j) which encodes the fact that sample i and sample j are connected
% by an edge. For each 
edges = zeros(max_tree - 1, 2);
edge_lengths = zeros(max_tree - 1, 1);

% i - this integer keeps track of the number of nodes we
% have in the graph so far
i = 1;
while i < max_tree
    % Note that we are assuming that RandomSample returns a sample in
    % freespace
    if randi(10) == 1
        x_sample = dest_config;
    else
        x_sample = RandomSample();
    end
    % Find the nearest node in the random tree to x_sample
    % at the end of this call the array distances will indicate the
    % distance between the new sample and each of the samples that has been
    % generated so far in the program.
    distances = Dist(x_sample,samples);
    [min_dist,current] = min(distances);
    x_nearst = samples(current,:);
    if LocalPlanner(x_sample, x_nearst)
        i = i + 1;     
        edges(i-1,:) = [current,i];
        edge_lengths(i-1,:) = min_dist;
        samples(i,:) = x_sample;
        if x_sample == dest_config
            break;
        end
    end
end

edges(i:end,:) = [];
edge_lengths(i:end,:) = [];
if i < max_tree
    samples(i+1:end,:) = [];
end

if (samples(end,:) ~= dest_config)
    path = 1;
else
    path = edges(end,:);
    
    while (edges(path(1)-1, 1) ~= 1)
        path = [edges(path(1)-1, 1), path];
    end
    path = [1,path];
end

searchtree.samples = samples;
searchtree.edges = edges;
searchtree.edge_lengths = edge_lengths;
searchtree.path = path;