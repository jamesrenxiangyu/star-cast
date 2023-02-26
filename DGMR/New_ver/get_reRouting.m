%% LPCA: Improved DGMR based on relay selection
function [new_relay, new_delay, new_relia, new_throu, new_cost] = get_reRouting(QoS_th, clusterMember, refRelay, CH_idx, Ch_par, refDistance)
%
% Input: QoS_th -- SNR, Data rate, per-hop and e2e reliability, delay threshold
%        clusterMember -- Locations and speed of all vehicles of the cluster. It has size
%                     N x 4, first column: v_id, loc_x, loc_y, speed
%        refRelay -- Original relay vehicles indices of shape M x 1
%        CH_idx -- Cluster head index
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: new_relay -- new relay vehicles (path)
%         new_delay -- per-hop delay
%         new_relia -- per-hop reliability
%         new_throu -- per-hop throughput
%         new_cost -- per-hop cost


new_relay = [];
new_delay = [];
new_relia = [];
new_throu = [];
new_cost = [];

% 'v_id, loc_x, loc_y, spd'
X = sortrows(clusterMember, 2); % sort locations by x-axis
loc = X(:,1:3);
spd = X(:,[1,4]);

% Coverage = max(loc(:,2)) - min(loc(:,1));
% h_loc = loc(find(loc(:,1)==CH_idx), 2:end);

RAT_th = QoS_th(2);
PDR_th = QoS_th(3);
REL_th = QoS_th(3);
DEL_th = QoS_th(4);

Pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant


% V2N2V delay metric
Cdelay = (2 * Pkt/RAT_th + tau) * 1e3; % in ms
Ccost = 2 * 5;
Vcost = 2 * 1;

dmin = refDistance(1); % distance lower bound 
dmax = refDistance(2); % distance upper bound

Th = 1;
Cost = 1;

%% Divide the cluster depending on the location of the CH
cflag1 = find(X(:,1)==CH_idx); % find the location of the CH in the cluster
cflag2 = find(refRelay==CH_idx); % find the index of the CH in the relay candidate list

% Bug: refRelay not in the cluster

if cflag1 == 1 % CH is at the back edge of the cluster
    l1 = loc;
    s1 = spd;
    if isempty(cflag2)
        r1 = [];
    else
        r1 = refRelay;
    end
    l2 = [];
    s2 = [];
    r2 = [];
elseif cflag1 == size(X,1) % CH is at the front edge of the cluster
    l1 = [];
    s1 = [];
    r1 = [];
    l2 = flip(loc);
    s2 = flip(spd);
    if isempty(cflag2)
        r2 = [];
    else
        r2 = flip(refRelay);
    end
else % CH is in the middle of the cluster
    l1 = loc(cflag1:end, :);
    s1 = spd(cflag1:end, :);
    if isempty(cflag2)
        r1 = [];
    else
        r1 = refRelay(cflag2:end, :);
    end
    
    l2 = flip(loc(1:cflag1,:));
    s2 = flip(spd(1:cflag1,:));
    if isempty(cflag2)
        r2 = [];
    else
        r2 = flip(refRelay(1:cflag2,:));
    end
end    
%% Execute relay selection for each hop (Whole path)

rCan1 = [];
Del1 = [];
Rel1 = [];
Rat1 = [];
Cost1 = [];

rCan2 = [];
Del2 = [];
Rel2 = [];
Rat2 = [];
Cost2 = [];

if ~isempty(l2) % Backward transmission
%     coverage2 = norm(max(l2(:,2)) - min(l2(:,2)));
    coverage2 = abs(max(l2(:,2)) - min(l2(:,2)));
    refDis = refDistance;
    rel_th = PDR_th;
    cov = 0;
    itr = 2;
    preidx = CH_idx;
    
    path = preidx;
    del_con = [];
    pdr_con = [];
    th_con = [];
    cost_con = [];
    
    dly_flag = 0;
    pdr_flag = 1;
    
    l = l2(find(l2(:,1) == preidx):end, :);
    s = s2(find(l2(:,1) == preidx):end, :);
    if size(l,1) == 1
        warning('Error');
    end
    
    % bug
    while cov < coverage2 || size(l,1) >1
        if isempty(r2)
            % Refrelay do not exist
            nxtidx = l(end,:);
        else
            if length(r2) == 1
                % only one hop
                nxtidx = r2(1);
            elseif length(r2) > 1
                % multiple hops
                nxtidx = r2(itr);
            end
        end 
        
        if isempty(l)
            warning('Error!')
            return
        end
        [rIdx, rDel, rPdr, rDis, rRat] = get_RelaySelect(rel_th, l, s, nxtidx, Ch_par, refDis, RAT_th, CH_idx);
        itr = itr + 1;
        preidx = rIdx;
        
        if isempty(r2)
            % Refrelay do not exist
            nxtidx = l(end,1);
        else
            if itr >= length(r2)
                nxtidx = r2(end);
            else
                nxtidx = r2(itr);
            end
        end 
        
        dly_flag = dly_flag + rDel;
        pdr_flag = pdr_flag * rPdr;
        cov = cov + rDis;
        d_min = (coverage2 - cov) * (Pkt + tau * RAT_th)/((DEL_th - dly_flag) * RAT_th);
        if d_min < 0
            d_min = 0;
        end
        
        refDis(1) = d_min;
        rel_th = REL_th/pdr_flag;
        if rel_th >= REL_th
            rel_th = REL_th;
        end
        path = [path, rIdx];
        del_con = [del_con, rDel];
        pdr_con = [pdr_con, rPdr];
        th_con = [th_con, rRat];
        cost_con = [cost_con, Vcost];
        
        if cov + dmax >= coverage2
            rIdx = l(end,1);
            gg = find(path == rIdx);
            if ~isempty(gg)
                break;
            end
            rDis = norm(l(find(l(:,1) == rIdx),2:end) - l(find(l(:,1) == preidx),2:end));
            [rPdr, rate, ~]= get_linkCal(rDis, Ch_par, RAT_th);
%             [rPdr, rate] = linkCal(rDis, Ch_par);
            rDel = Pkt/rate + tau;
            path = [path, rIdx];
            del_con = [del_con, rDel];
            pdr_con = [pdr_con, rPdr];
            th_con = [th_con, rate];
            cost_con = [cost_con, Vcost];
            break;
        end
    end
    rCan1 = [rCan1; flip(path)];
    Del1 = [Del1, del_con] .* 1e3;
    Rel1 = [Rel1, pdr_con];
    Rat1 = [Rat1, th_con];
    Cost1 = [Cost1, cost_con];
end

if ~isempty(l1) % Forward transmission
    coverage1 = abs(max(l1(:,2)) - min(l1(:,2)));
    refDis = refDistance;
    rel_th = PDR_th;
    cov = 0;
    itr = 2;
    preidx = CH_idx;

    path = preidx;
    del_con = [];
    pdr_con = [];
    th_con = [];
    cost_con = [];
    
    dly_flag = 0;
    pdr_flag = 1;

    l = l1(find(l1(:,1) == preidx):end, :);
    s = s1(find(l1(:,1) == preidx):end, :);

    if size(l,1) == 1
        warning('Error');
    end
    % bug
    while cov < coverage1 || size(l,1) > 1
        if isempty(r1)
            % Refrelay do not exist
            nxtidx = l(end,1);
        else
            if length(r1) == 1
                % only one hop
                nxtidx = r1(1);
            elseif length(r1) > 1
                % multiple hops
                nxtidx = r1(itr);
            end
        end 
        
        if isempty(l)
            warning('Error!')
            return;
        end
        
%         if size(l,1) == 1 && l(1,1) == nxtidx(1,1)
%             % This is last hop
%             rIdx = nxtidx(1,1)
        
        [rIdx, rDel, rPdr, rDis, rRat] = get_RelaySelect(rel_th, l, s, nxtidx, Ch_par, refDis, RAT_th, CH_idx);
        itr = itr + 1;
        preidx = rIdx;
        
        if isempty(r1)
            % Refrelay do not exist
            nxtidx = l(end,:);
        else
            if itr >= length(r1)
                nxtidx = r1(end);
            else
                nxtidx = r1(itr);
            end
        end 

        dly_flag = dly_flag + rDel;
        pdr_flag = pdr_flag * rPdr;
        cov = cov + rDis;
        d_min = (coverage1 - cov) * (Pkt + tau * RAT_th)/((DEL_th - dly_flag) * RAT_th);
        if d_min < 0
            d_min = 0;
        end
        refDis(1) = d_min;
        rel_th = REL_th/pdr_flag;
        if rel_th >= REL_th
            rel_th = REL_th;
        end
        path = [path, rIdx];
        del_con = [del_con, rDel];
        pdr_con = [pdr_con, rPdr];
        th_con = [th_con, rRat];
        cost_con = [cost_con, Vcost];
        
        if cov + dmax >= coverage1
            rIdx = l(end,1);
            gg = find(path == rIdx);
            if ~isempty(gg)
                break;
            end
            rDis = norm(l(find(l(:,1) == rIdx),2:end) - l(find(l(:,1) == preidx),2:end));
            [rPdr, rate, ~]= get_linkCal(rDis, Ch_par, RAT_th);
            rDel = Pkt/rate + tau;
            path = [path, rIdx];
            del_con = [del_con, rDel];          
            pdr_con = [pdr_con, rPdr];
            th_con = [th_con, rate];
            cost_con = [cost_con, Vcost];
            break;
        end
    end
    rCan2 = [rCan2; path];
    Del2 = [Del2, del_con] .* 1e3; % in ms
    Rel2 = [Rel2, pdr_con];
    Rat2 = [Rat2, th_con];
    Cost2 = [Cost2, cost_con];
end

new_relay1 = [];
new_relay2 = [];


for i = 1: length(rCan1)
    new_relay1 = [new_relay1; clusterMember(clusterMember(:,1) == rCan1(i), :)];
end

for i = 1: length(rCan2)
    new_relay2 = [new_relay2; clusterMember(clusterMember(:,1) == rCan2(i), :)];
end

new_relay = {new_relay2; new_relay1};

% 
% if isempty(rCan1)
%     Del1 = inf * ones(1, length(Del2));
%     Rel1 = 0 * ones(1, length(Del2));
%     Rat1 = 0 * ones(1, length(Del2));
%     Cost1 = inf * ones(1, length(Del2));
% end
% 
% if isempty(rCan2)
%     Del2 = inf * ones(1, length(Del1));
%     Rel2 = 0 * ones(1, length(Del1),1);
%     Rat2 = 0 * ones(1, length(Del1));
%     Cost2 = inf * ones(1, length(Del1));
% end
% 

new_delay = {Del2; Del1};
new_relia = {Rel2; Rel1};
new_throu = {Rat2; Rat1};
new_cost = {Cost2; Cost1};


% if sum(Del1) == 0 || sum(Del2) == 0
%     Del = max(sum(Del1), sum(Del2));
%     Rel = max(prod(Rel1), prod(Rel2));
% else
%     Del = max(sum(Del1), sum(Del2));
%     Rel = min(prod(Rel1), prod(Rel2));
% end




