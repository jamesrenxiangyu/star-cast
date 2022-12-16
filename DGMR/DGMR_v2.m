% heurstic relay selection at each hop 
function [rCAN, rPDR, rRAT, rREL] = DGMR_v2(QoS_th, Locations, Coverage, Ch_par, refDistance)
%
% Input: Locations of vehicles,
%        Channel model parameters, per hop delay constant
%        SNR, Data rate, per-hop and e2e reliability, delay threshold
%        Coverage requirement
% 
% Output: Relay locations, per-hop PDR, e2e reliability, 
% 

SNR_th = QoS_th(1);
RAT_th = QoS_th(2);
PDR_th = QoS_th(3);
REL_th = QoS_th(4);
DEL_th = QoS_th(5);

loc = Locations;

m = Ch_par(1); % Modulation
pt_max = Ch_par(2); % Tx power in mW
b = Ch_par(3); % Channel bandwidth
alpha = Ch_par(4); % Path loss component
N0 = Ch_par(5); % Noise power level in mW
Xg = Ch_par(6); % Shadowing component
f = Ch_par(7); % Radio frequency in GHz
pkt = Ch_par(8); % Packet length in bit
tau = Ch_par(9); % Delay constant

d_min = refDistance(1);
d_max = refDistance(2);

PDR_re = PDR_th;
src = loc(1);
rCAN = [];
rPDR = [];
rRAT = [];
rREL = 0;
dist_flag = 0;
pdr_flag = 1;


while dist_flag < Coverage
    loc_temp = loc - src;
    rSpace = find(loc_temp >= d_min & loc_temp <= d_max);
    if isempty(rSpace)
        disp('error1: No candidate, add hops');
        rPick = find(loc_temp < d_min, 1, 'last');
        r = loc_temp(rPick);
        [pdr, rate] = linkCal(r, Ch_par);
    else
        rSize = size(rSpace,2);
        rPick = randi(rSize); % random selection
%         display(rPick);
        r = loc_temp(rSpace(rPick));
        [pdr, rate] = linkCal(r, Ch_par);
        while pdr < PDR_re 
            rPick = rPick - 1;
            if rPick < 1
                disp('error2: No candidate');
                rPick = 1;
            break;
            end
            r = loc_temp(rSpace(rPick));
            [pdr, rate] = linkCal(r, Ch_par);
%             if pdr >= PDR
%                 pdr_flag = pdr_flag/pdrr;
%                 pdr_flag = pdr * pdr_flag;
%             end
        end
    end    
    dly_flag = pkt/rate + tau;
    r = r + src; % map to actual location
    pdr_flag = pdr * pdr_flag;
    rCAN = [rCAN; r];
    rPDR = [rPDR; pdr];
    rRAT = [rRAT; rate];
    
    cov = Coverage - r;
    d_min = cov * (pkt + tau * RAT_th)/((DEL_th - dly_flag) * RAT_th);
    if d_min > d_max
       disp('Service Error');
       break;
    end
    
    PDR_re = REL_th/pdr_flag;
    if PDR_re > PDR_th
       PDR_re = PDR_th;
    end
   
    src = r;
    dist_flag = r + d_max;
    if dist_flag >= Coverage
        loc_temp = loc - src;
        r = loc_temp(end);
        [pdr, rate] = linkCal(r, Ch_par);
        r = r + src;
        if r ~= rCAN
            rCAN = [rCAN; r];
            rPDR = [rPDR; pdr];
            rRAT = [rRAT; rate];
        end
    end  
end
rREL = prod(rPDR);


    

