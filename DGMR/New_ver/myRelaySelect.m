%% Improved heurstic relay selection at each hop 
function [rIdx, rDel, rPdr, rDis] = myRelaySelect(Rel_th, Locations, Speed, refRelay, Ch_par, refDistance)
%
% Input: Rel_th -- Reliability threshold
%        Locations -- Locations of one-hop vehicles with index, has size
%                     N x 3, first column is the index, rest are the
%                     x and y locations. First row is the CH. Note that the
%                     locations should be in ascending order by x-axis (1 col)
%        Speed -- Speed of one-hop vehicles, of size N x 1
%        refRelay -- Reference relay vehicle's index
%        Ch_par -- Channel model parameters, per hop delay constant
%        refDistance -- per-hop transmission upper and lower bound
% 
% Output: rIdx -- relay index
%         rDel -- estimated per-hop transmission delay
%         rPdr -- estimated per-hop packet delivery ratio

loc = Locations;
spd = Speed;
ridx = refRelay;

pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant

d_min = refDistance(1);
d_max = refDistance(2);

loc_src = loc(1,2:end);
spd_src = spd(1,2);

%  vehicles within reference distance
rSpace = find(abs(loc(:,2)-loc_src(1,1)) >= d_min & abs(loc(:,2)-loc_src(1,1))<= d_max);
if isempty(rSpace)
    disp('error1: No candidate, add hops');
    rPick = find(abs(loc(:,2)-loc_src(1,1)) < d_min, 1, 'last');
    tarloc = loc(rPick, 2:end);
    dist = norm(tarloc-loc_src);
    [pdr, rate] = linkCal(dist, Ch_par);
    while pdr < Rel_th
        rPick = rPick - 1;
        if rPick == 0
            rPick = 1;
            break;
        end
        tarloc = loc(rPick, 2:end);
        dist = norm(tarloc-loc_src);
        [pdr, rate] = linkCal(dist, Ch_par);
    end    
else
    if isempty(find(loc(:,1) == ridx))% Previous relay not within rSpace
        disp('Relay out! Adjust relay');
        for k = 1:length(rSpace)
            rl = loc(rSpace(k),2:end);
            rs = spd(rSpace(k),2);
            resTime(k) = myLinklife(loc_src, spd_src, rl, rs, refDistance);
        end
        [~, rPick] = max(resTime);
        tarloc = loc(rSpace(rPick), 2:end);
        dist = norm(tarloc-loc_src);
        [pdr, rate] = linkCal(dist, Ch_par);
        while pdr < Rel_th
            rPick = rPick - 1;
            if rPick == 0
                rPick = 1;
                break;
            end
            tarloc = loc(rSpace(rPick), 2:end);
            dist = norm(tarloc);
            [pdr, rate] = linkCal(dist, Ch_par);
        end
        ridx = loc(rSpace(rPick),1);
    elseif sum(rSpace == find(loc(:,1) == ridx)) >= 1   % previous relay is within rSpace
        tarloc = loc(find(loc(:,1) == ridx), 2:end); % location of ref vehicle
        % check reliability requirement
        dist = norm(tarloc-loc_src);
        [pdr, rate] = linkCal(dist, Ch_par);
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
                dist = norm(tarloc-loc_src);
                [pdr, rate] = linkCal(dist, Ch_par);
            end
            ridx = loc(rSpace(rPick),1);
        else
            % Check residual link lifetime  
            tarspd = spd(find(loc(:,1) == ridx), 2:end);
            tarResTime = myLinklife(loc_src, spd_src, tarloc, tarspd, refDistance);
            % Compute average residual link lifetime of relay candidates
            resTime = zeros(length(rSpace),1);
            for k = 1:length(rSpace)
                rl = loc(rSpace(k),2:end);
                rs = spd(rSpace(k),2);
                resTime(k) = myLinklife(loc_src, spd_src, rl, rs, refDistance);
            end
            avgTime = sum(resTime)/length(rSpace);
            if tarResTime < avgTime
                % Previous relay has small residual link lifetime
                disp('ResTime out! Adjust relay');
                [~, rPick] = max(resTime);
                tarloc = loc(rSpace(rPick), 2:end);
                dist = norm(tarloc-loc_src);
                [pdr, rate] = linkCal(dist, Ch_par);
                while pdr < Rel_th
                    rPick = rPick - 1;
                    if rPick == 0
                        rPick = 1;
                        break;
                    end
                    tarloc = loc(rSpace(rPick), 2:end);
                    dist = norm(tarloc-loc_src);
                    [pdr, rate] = linkCal(dist, Ch_par);
                end
                ridx = loc(rSpace(rPick),1);
            end  
        end
    else
        disp('Relay out! Adjust relay');
        for k = 1:length(rSpace)
            rl = loc(rSpace(k),2:end);
            rs = spd(rSpace(k),2);
            resTime(k) = myLinklife(loc_src, spd_src, rl, rs, refDistance);
        end
        [~, rPick] = max(resTime);
        tarloc = loc(rSpace(rPick), 2:end);
        dist = norm(tarloc-loc_src);
        [pdr, rate] = linkCal(dist, Ch_par);
        while pdr < Rel_th
            rPick = rPick - 1;
            if rPick == 0
                rPick = 1;
                break;
            end
            tarloc = loc(rSpace(rPick), 2:end);
            dist = norm(tarloc);
            [pdr, rate] = linkCal(dist, Ch_par);
        end
        ridx = loc(rSpace(rPick),1);
    end
end
rIdx = ridx;
rPdr = pdr;
rDel = pkt/rate + tau; % in ms
rDis = abs(loc(find(loc(:,1) == rIdx), 2) - loc_src(1,1));





