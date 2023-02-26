function cluster = get_myKMeans(K, L, T, M)

%kMeans Clusters data points into k clusters.
%   Input args: K: Number of clusters; 
%               P: Positions, m-by-n matrix of n m-dimensional data points.
%               S: Speed, vector of size n.
%               T: Time gap of each sampling.
%               E = 0.2: Epsilon-greedy for centroid update
%               M: Mode selection, M=1 use revised distance, M=2 use
%               euclidean distance
%   Output args: K x 2 cell 
%               cluster: member information of each cluster (v_id, v_lov, v_spd)
%               centr: CH information 
%   representing in which cluster the corresponding point lies in
%   centr: m-by-k matrix of the m-dimensional centroids of the k clusters

E = 0.2;
P = L(:,1:3)';
S = L(:,4)';

cluster = cell(K, 2);

if size(P,2) < 2
    warning("Single member cluster!")
    cluster{1,1} = L;
    cluster{1,2} = L';
    return
end

numP = size(P,2); % number of points
dimP = size(P,1); % dimension of points

% Initialize points plot
% figure(1);
% scatter(P(1,:), ones(numP,1));
% hold on;

%% Choose K data points as initial centroids

% choose k unique random indices between 1 and size(P,2) (number of points)
randIdx = randperm(numP,K);
% randIdx = ceil(linspace(1, numP, K));

% initial centroids
centr = P(:,randIdx);
centrS = S(randIdx);

% figure(1);
% scatter(centr(1,:), ones(length(centr),1),'xk','LineWidth',1.5);

%% Repeat until stopping criterion is met

% init cluster array
cluster_all = [zeros(1,numP); P];

% init previous cluster array clusterPrev (for stopping criterion)
clusterPrev = cluster_all;

% for reference: count the iterations
iterations = 0;

% init stopping criterion
stop = false; % if stopping criterion met, it changes to true

while stop == false
    
    % init array to store distance results
    distP = zeros(numP,1);
    % for each data point
    for idxP = 1:numP
        % init distance array dist
        dist = zeros(1,K);
        % compute distance to each centroid
        for idxC=1:K
            if M == 1
                dist(idxC) = sqrt((norm(P(2:3,idxP) - centr(2:3,idxC)))^2 + ...
                    (norm(P(2:3,idxP) + S(idxP)*T - (centr(2:3,idxC) + centrS(idxC)* T)))^2);
            elseif M == 2
                dist(idxC) = norm(P(2:3,idxP)-centr(2:3,idxC));
            else
                warning("Wrong mode selection!")
            end
        end
        % find index of closest centroid (= find the cluster)
        [mdist, clusterP] = min(dist);
        distP(idxP) = mdist;
        cluster_all(1,idxP) = clusterP;
    end
    
    % Recompute centroids using current cluster memberships:
        
    % init centroid array centr
    centr = zeros(dimP,K);
    % for every cluster compute new centroid
    for idxC = 1:K
        % find the points in cluster number idxC and compute row-wise mean
%         centr(:,idxC) = mean(P(:,cluster==idxC),2);
        % update the centroid from existing data points using probability
          tempC = P(:,cluster_all(1,:)==idxC); % position of the cluster members
          tempD = distP(cluster_all(1,:)==idxC); % distance value of the cluster members
          Cprob = tempD./(sum(tempD));
        % centroid selection with epsilon-greedy
          CDFprob = Cprob' * triu(ones(length(Cprob)));
          rnd = rand(1);
          flag = find(CDFprob > rnd, 1);
%           rnd = rand(1);
%           if rnd < E && length(tempD) > 2
%               flag = randi(1,size(tempD,2));
%           else
%               CDFprob = Cprob' * triu(ones(length(Cprob)));
%               flag = find(CDFprob > rnd, 1);
% %               [~, flag] = max(Cprob);
%           end
          centr(:,idxC) = tempC(:,flag);
    end
    
    % Checking for stopping criterion: Clusters do not change anymore
    if clusterPrev==cluster_all
        stop = true;
    end
    % update previous cluster clusterPrev
    clusterPrev = cluster_all;
    
    iterations = iterations + 1;
    
%     hold on;
%     scatter(centr(1,:), ones(length(centr),1),'xk','LineWidth',1.5);
%     pause(2)
end

% organize output
cluster_all = [cluster_all; S]';
centr = sortrows(centr');
cluster_id = unique(cluster_all(:,1), 'stable');
for i = 1:K
    cluster_mem = cluster_all( cluster_all(:,1) == cluster_id(i), 2:end);
    cluster{i,1} = cluster_mem;
    cluster{i,2} = centr(i,:);
end

    

% for reference: print number of iterations
fprintf('kMeans.m used %d iterations of changing centroids.\n',iterations);
end