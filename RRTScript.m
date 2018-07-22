%
% RRTScript.
%

%% read obstacles

close all;
format long
di = dir('obstacles.csv');
obstacles = csvread(di.name,5);

%% Build the tree

max_tree = 200;
start_config = [-0.5,-0.5];
dest_config  = [ 0.5, 0.5];
searchtree = RRT(@()(RandomSample(obstacles)), @Dist, @(x,y)(LocalPlanner(x,y,obstacles)), max_tree, start_config, dest_config);

csvwrite('edges.csv',[searchtree.edges,searchtree.edge_lengths]);
csvwrite('path.csv',searchtree.path);
csvwrite('nodes.csv',[[1:length(searchtree.samples)]',[searchtree.samples]]);
