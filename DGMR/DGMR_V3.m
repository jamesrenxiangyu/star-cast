%% LPCA: Improved DGMR based on relay selection
function [rCan, Del, Rel] = DGMR_V3(QoS_th, Locations, Speed, refRelay, CH_idx, Ch_par, refDistance)
%
% Input: QoS_th -- SNR, Data rate, per-hop and e2e reliability, delay threshold
%        Locations -- Locations of all vehicles of the cluster. It has size
%                     N x 3, first column is the index, rest are the
%                     x and y locations.  
%        Speed -- Speed of all vehicles, of shape N x 1
%        refRelay -- Original relay vehicles indices of shape N x 1
%        CH_idx -- Cluster head index
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: rCan -- relay vehicles (path)
%         Del -- end-to-end delay
%         Rel -- end-to-end reliability

locRaw = Locations;
X = [locRaw, Speed(:,2)];
X = sortrows(X, 2); % sort locations by x-axis
loc = X(:, 1:3);
spd = X(:, [1 4]);
% Coverage = max(loc(:,2)) - min(loc(:,1));
% h_loc = loc(find(loc(:,1)==CH_idx), 2:end);

RAT_th = QoS_th(2);
PDR_th = QoS_th(3);
REL_th = QoS_th(4);
DEL_th = QoS_th(5);

pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant

dmin = refDistance(1); % distance lower bound 
dmax = refDistance(2); % distance upper bound



%% Divide the cluster depending on the location of the CH
cflag1 = find(loc(:,1)==CH_idx); % find the location of the CH in the cluster
cflag2 = find(refRelay(:,1)==CH_idx); % find the index of the CH in the relay candidate list


if cflag1 == 1 % CH is at the back edge of the cluster
    l1 = loc;
    s1 = spd;
    r1 = refRelay;
    l2 = [];
    s2 = [];
    r2 = [];
elseif cflag1 == size(loc,1) % CH is at the front edge of the cluster
    l1 = [];
    s1 = [];
    r1 = [];
    l2 = flip(loc);
    s2 = flip(spd);
    r2 = flip(refRelay);
else % CH is in the middle of the cluster
    l1 = loc(cflag1:end, :);
    s1 = spd(cflag1:end, :);
    r1 = refRelay(cflag2:end, :);
    l2 = flip(loc(1:cflag1,:));
    s2 = flip(spd(1:cflag1,:));
    r2 = flip(refRelay(1:cflag2,:));
end    
%% Execute relay selection for each hop (Whole path)

rCan1 = [];
Del1 = [];
Rel1 = [];
rCan2 = [];
Del2 = [];
Rel2 = [];

if ~isempty(l2) % Backward transmission
    coverage2 = abs(max(l2(:,2)) - min(l2(:,2)));
    refDis = refDistance;
    rel_th = PDR_th;
    cov = 0;
    itr = 2;
    preidx = CH_idx;
    if length(r2) == 1
        nxtidx = r2(1);
    else
        nxtidx = r2(itr);
    end
    path = preidx;
    del_con = [];
    pdr_con = [];
    
    dly_flag = 0;
    pdr_flag = 1;
    while cov < coverage2
        l = l2(find(l2(:,1) == preidx):end, :);
        s = s2(find(l2(:,1) == preidx):end, :);
        [rIdx, rDel, rPdr, rDis] = myRelaySelect(rel_th, l, s, nxtidx, Ch_par, refDis);
        itr = itr + 1;
        preidx = rIdx;
        if itr >= length(r2)
            nxtidx = r2(end);
        else
            nxtidx = r2(itr);
        end
        dly_flag = dly_flag + rDel;
        pdr_flag = pdr_flag * rPdr;
        cov = cov + rDis;
        d_min = (coverage2 - cov) * (pkt + tau * RAT_th)/((DEL_th - dly_flag) * RAT_th);
        refDis(1) = d_min;
        rel_th = REL_th/pdr_flag;
        if rel_th >= REL_th
            rel_th = REL_th;
        end
        path = [path, rIdx];
        del_con = [del_con, rDel];
        pdr_con = [pdr_con, rPdr];
        
        if cov + dmax >= coverage2
            rIdx = l(end,1);
            gg = find(path == rIdx);
            if ~isempty(gg)
                break;
            end
            rDis = norm(l(find(l(:,1) == rIdx),2:end) - l(find(l(:,1) == preidx),2:end));
            [rPdr, rate] = linkCal(rDis, Ch_par);
            rDel = pkt/rate + tau;
            path = [path, rIdx];
            del_con = [del_con, rDel];
            pdr_con = [pdr_con, rPdr];
            break;
        end
    end
    rCan1 = [rCan1, flip(path)];
    Del1 = [Del1, del_con];
    Rel1 = [Rel1, pdr_con];
end

if ~isempty(l1) % Forward transmission
    coverage1 = abs(max(l1(:,2)) - min(l1(:,2)));
    refDis = refDistance;
    rel_th = PDR_th;
    cov = 0;
    itr = 2;
    preidx = CH_idx;
    if length(r1) == 1
        nxtidx = r1(1);
    else
        nxtidx = r1(itr);
    end
    if isempty(rCan1)
        path = preidx;
    else
        path = [];
    end
    del_con = [];
    pdr_con = [];
    
    dly_flag = 0;
    pdr_flag = 1;
    while cov < coverage1
        l = l1(find(l1(:,1) == preidx):end, :);
        s = s1(find(l1(:,1) == preidx):end, :);
        [rIdx, rDel, rPdr, rDis] = myRelaySelect(rel_th, l, s, nxtidx, Ch_par, refDis);
        itr = itr + 1;
        preidx = rIdx;
        if itr >= length(r1)
            nxtidx = r1(end);
        else
            nxtidx = r1(itr);
        end
        dly_flag = dly_flag + rDel;
        pdr_flag = pdr_flag * rPdr;
        cov = cov + rDis;
        d_min = (coverage1 - cov) * (pkt + tau * RAT_th)/((DEL_th - dly_flag) * RAT_th);
        refDis(1) = d_min;
        rel_th = REL_th/pdr_flag;
        if rel_th >= REL_th
            rel_th = REL_th;
        end
        path = [path, rIdx];
        del_con = [del_con, rDel];
        pdr_con = [pdr_con, rPdr];
        
        if cov + dmax >= coverage1
            rIdx = l(end,1);
            gg = find(path == rIdx);
            if ~isempty(gg)
                break;
            end
            rDis = norm(l(find(l(:,1) == rIdx),2:end) - l(find(l(:,1) == preidx),2:end));
            [rPdr, rate] = linkCal(rDis, Ch_par);
            rDel = pkt/rate + tau;
            path = [path, rIdx];
            del_con = [del_con, rDel];
            pdr_con = [pdr_con, rPdr];
            break;
        end
    end
    rCan2 = [rCan2, path];
    Del2 = [Del2, del_con];
    Rel2 = [Rel2, pdr_con];
end

rCan = [rCan1, rCan2];
if sum(Del1) == 0 || sum(Del2) == 0
    Del = max(sum(Del1), sum(Del2));
    Rel = max(prod(Rel1), prod(Rel2));
else
    Del = max(sum(Del1), sum(Del2));
    Rel = min(prod(Rel1), prod(Rel2));
end



