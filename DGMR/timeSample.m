%% MBS sample vehicle mobility information 
clear; clc;
close all;

% sample data every 10s
sampRate = 10;

path = 'SUMO/';
locfile = 'Locations.mat';
mobfile = 'Speed.mat';

load([path locfile])
load([path mobfile])

% Reserve data with target vehicle (id=1)
tar_id1 = find(Pos_x(:,1) == 1); % locate target vehicle
tar_id2 = find(Pos_x(:,1) ~= 0, 1, 'Last'); % count total vehicle number
tar_begin = find(Pos_x(tar_id1, 2:end) ~= 0, 1); % time target vehicle enters the network
tar_end = find(Pos_x(tar_id1, :) ~= 0, 1, 'Last'); % time target vehicle exits the network

% retrieve gloable network information over time involving target vehicle
loc_x = [Pos_x(1:tar_id1, 1) Pos_x(1:tar_id1, tar_begin:tar_end)];
loc_y = [Pos_y(1:tar_id1, 1) Pos_y(1:tar_id1, tar_begin:tar_end)];
mob = [Speed(1:tar_id1, 1) Speed(1:tar_id1, tar_begin:tar_end)]; 


LOC_x = loc_x(:,1:sampRate:end);
LOC_y = loc_y(:,1:sampRate:end);
MOB = mob(:,1:sampRate:end);

flag = 50;
X = [LOC_x(2:end, flag), LOC_y(2:end, flag)]; % Use current time
X = sortrows(X,1);
xmin = min(X(:,1));
F = [LOC_x(2:end, flag+1), LOC_y(2:end, flag+1)]; % Test with next time
F = sortrows(F,1);



%% test of myKmeans.m
K = 4;
S = MOB(2:end, flag);
T = sampRate;
E = 0.2;
M = 1; % mode selection 


[cluster1, centr1] = myKMeans(K, X', S, T, M);
figure(1);
for i = 1:size(centr1,2)
    temp = X(cluster1 == i,:);
    subplot(2,1,1)
    scatter(temp(:, 1)-xmin, temp(:,2), 'o');
    hold on;
end
subplot(2,1,1)
scatter(centr1(1,:)-xmin, centr1(2,:), 'kx', 'LineWidth', 1.5)
title("Kmeans with revised distance");
subplot(2,1,2)
scatter(F(:,1)-xmin, F(:,2));

[cluster2, centr2] = myKMeans(K, X', S, T, 2);
figure(2);
for i = 1:size(centr2,2)
    temp = X(cluster2 == i,:);
    subplot(2,1,1)
    scatter(temp(:, 1)-xmin, temp(:,2), 'o');
    hold on;
end
subplot(2,1,1)
scatter(centr2(1,:)-xmin, centr2(2,:), 'kx', 'LineWidth', 1.5)
title("Kmeans with Euclidean distance");
subplot(2,1,2)
scatter(F(:,1)-xmin, F(:,2));


[ind, ctr] = kmeans(X, K);
figure(3);
for i = 1:size(ctr,1)
    temp = X(ind == i,:);
    subplot(2,1,1)
    scatter(temp(:, 1)-xmin, temp(:,2), 'o');
    hold on;
end
subplot(2,1,1)
scatter(ctr(:,1)-xmin, ctr(:,2), 'rx', 'LineWidth', 1.5)
title("Kmeans by default");
subplot(2,1,2)
scatter(F(:,1)-xmin, F(:,2));

% 
% Y = [LOC_x(2:end, flag+1), LOC_y(2:end, flag+1)];
% scatter(Y(:,1), Y(:,2))

