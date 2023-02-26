%% Improved heurstic relay selection at each hop 
function [rIdx, rDel, rPdr, rDis, rRat] = get_RelaySelect(Rel_th, Locations, Speed, refRelay, Ch_par, refDistance, RAT_th, CH_idx)
%
% Input: Rel_th -- Reliability threshold
%        Locations -- Locations of one-hop vehicles with index, has size
%                     N x 3, first column is the index, rest are the
%                     x and y locations. First row is the CH. Note that the
%                     locations should be in ascending order by x-axis (1 col)
%        Speed -- Speed of one-hop vehicles, of size N x 1
%        refRelay -- Reference relay vehicle's v_id
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: rIdx -- relay index
%         rDel -- estimated per-hop transmission delay
%         rPdr -- estimated per-hop packet delivery ratio
%         rDis -- distance to the new relay  
%         rRat -- data rate achieved by the new relay  

loc = Locations;
spd = Speed;
ridx = refRelay(1,1);

pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9) * 1e3; % Delay constant in ms

d_min = refDistance(1);
d_max = refDistance(2);

% Bug: choose the other end of the cluster
if isempty(loc)
    warning('Error!')
else
    loc_src = loc(1,2:end);
%     if loc_src(1) == CH_idx
%         loc_src = loc(end,2:end);
%     end
end
        
spd_src = spd(1,2);




%  vehicles within reference distance
vec_vec = loc(:,2:3) - loc_src;
dist_vec = sqrt(vec_vec(:,1).^2 + vec_vec(:,2).^2);
rSpace = find(dist_vec > d_min & dist_vec <= d_max);
% bug: inter vehicle distance larger than d_max

if isempty(rSpace)
    
    if max(dist_vec) > d_max
        % candidates are out of Tx distance range
        rPick = find(dist_vec >= d_max, 1);
        disp('error1: No candidate, add hops');
        tarloc = loc(rPick, 2:end);
        dis = norm(tarloc-loc_src);
        [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
    else
        % candidates are within Tx distance range
        rPick = find(dist_vec <= d_min, 1, 'last');
        tarloc = loc(rPick, 2:end);
        dis = norm(tarloc-loc_src);
        [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        while pdr < Rel_th
            rPick = rPick - 1;
            if rPick == 0
                rPick = 1;
                break;
            end
            tarloc = loc(rPick, 2:end);
            dis = norm(tarloc-loc_src);
            [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        end
    end
else
    if isempty(find(loc(:,1) == ridx))% Previous relay not within rSpace
        disp('Relay out! Adjust relay');
        for k = 1:length(rSpace)
            rl = loc(rSpace(k),2:end);
            rs = spd(rSpace(k),2);
            resTime(k) = get_Linklife(loc_src, spd_src, rl, rs, refDistance);
        end
        [~, rPick] = max(resTime);
        tarloc = loc(rSpace(rPick), 2:end);
        dis = norm(tarloc-loc_src);
        [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        while pdr < Rel_th
            rPick = rPick - 1;
            if rPick == 0
                rPick = 1;
                break;
            end
            tarloc = loc(rSpace(rPick), 2:end);
            dis = norm(tarloc - loc_src);
            [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        end
        ridx = loc(rSpace(rPick),1);
    elseif sum(rSpace == find(loc(:,1) == ridx)) >= 1   % previous relay is within rSpace
        tarloc = loc(find(loc(:,1) == ridx), 2:end); % location of ref vehicle
        % check reliability requirement
        dis = norm(tarloc-loc_src);
        [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        if pdr < Rel_th
            % Previous relay vehicle cannot support reliability
            disp('Reliability out! Adjust relay')
            rPick = find(loc(:,2) == tarloc(1), 1);
            while pdr < Rel_th
                rPick = rPick - 1;
                if rPick == 0
                    rPick = 1;
                    break;
                end
                tarloc = loc(rSpace(rPick), 2:end);
                dis = norm(tarloc-loc_src);
                [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
            end
            ridx = loc(rSpace(rPick),1);
        else
            % Check residual link lifetime  
            tarspd = spd(find(loc(:,1) == ridx), 2:end);
            tarResTime = get_Linklife(loc_src, spd_src, tarloc, tarspd, refDistance);
            % Compute average residual link lifetime of relay candidates
            resTime = zeros(length(rSpace),1);
            for k = 1:length(rSpace)
                rl = loc(rSpace(k),2:end);
                rs = spd(rSpace(k),2);
                resTime(k) = get_Linklife(loc_src, spd_src, rl, rs, refDistance);
            end
            avgTime = sum(resTime)/length(rSpace);
            if tarResTime < avgTime
                % Previous relay has small residual link lifetime
                disp('ResTime out! Adjust relay');
                [~, rPick] = max(resTime);
                tarloc = loc(rSpace(rPick), 2:end);
                dis = norm(tarloc-loc_src);
                [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
                while pdr < Rel_th
                    rPick = rPick - 1;
                    if rPick == 0
                        rPick = 1;
                        break;
                    end
                    tarloc = loc(rSpace(rPick), 2:end);
                    dis = norm(tarloc-loc_src);
                    [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
                end
                ridx = loc(rSpace(rPick),1);
            end  
        end
    else
        disp('Relay out! Adjust relay');
        for k = 1:length(rSpace)
            rl = loc(rSpace(k),2:end);
            rs = spd(rSpace(k),2);
            resTime(k) = get_Linklife(loc_src, spd_src, rl, rs, refDistance);
        end
        [~, rPick] = max(resTime);
        tarloc = loc(rSpace(rPick), 2:end);
        dis = norm(tarloc-loc_src);
        [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        while pdr < Rel_th
            rPick = rPick - 1;
            if rPick == 0
                rPick = 1;
                break;
            end
            tarloc = loc(rSpace(rPick), 2:end);
            dis = norm(tarloc - loc_src);
            [pdr, rate, ~]= get_linkCal(dis, Ch_par, RAT_th);
        end
        ridx = loc(rSpace(rPick),1);
    end
end
rIdx = ridx;
rPdr = pdr;
rDel = pkt/rate + tau; % in ms
rDis = dis;
rRat = rate;





