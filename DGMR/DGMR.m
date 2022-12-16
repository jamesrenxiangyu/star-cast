function [rCAN, rPDR, rRAT, rREL] = DGMR(SNR_th, RAT_th, PDR_th, REL_th, DEL_th, Locations, Coverage, ...
    Modulation, TxPower, Bandwidth, Alpha, Noise, Shadow, Frequency, PktLength, Tau)

pt_max = TxPower; % Tx power in mW
m = Modulation; % Modulation
f = Frequency; % Radio frequency in GHz
alpha = Alpha; % Path loss component
N0 = Noise; % Noise power level in mW
Xg = Shadow; % Shadowing component
pkt = PktLength; % Packet length in bit
b = Bandwidth; % Channel bandwidth
tau = Tau; % Delay constant
cov = Coverage;

PDR = PDR_th;
REL = REL_th;
DEL = DEL_th;
loc = Locations;
src = loc(1);
rCAN = [];
rPDR = [];
rRAT = [];
rREL = 0;
dist_flag = 0;
pdr_flag = 1;

% Initial Reference distance
dSNR = distSNR(SNR_th, m, pt_max, alpha, N0, Xg, f);
dRate = distRate(RAT_th, b, m, pt_max, alpha, N0, Xg, f);
dDel = cov * (pkt + tau * RAT_th)/(DEL_th * RAT_th);

d_min = dDel
d_max = min(dSNR, dRate)


if d_min > d_max
    disp('Service Error');
end

while dist_flag < Coverage
    loc_temp = loc - src;
    rSpace = find(loc_temp >= d_min & loc_temp <= d_max);
    if isempty(rSpace)
        disp('error1: No candidate, add hops');
    else
        rSize = size(rSpace,2);
        rPick = randi(rSize); % random selection
%         display(rPick);
        r = loc_temp(rSpace(rPick)); 
        [pdr, rate] = linkCal(r, m, pt_max, b, alpha, N0, Xg, f, pkt);
        pdr_flag = pdr * pdr_flag;
        while pdr < PDR 
%             pdrr = pdr;
            rPick = rPick - 1;
            if rPick < 1
                disp('error2: No candidate');
                break;
            end
            r = loc_temp(rSpace(rPick));
            [pdr, rate] = linkCal(r, m, pt_max, b, alpha, N0, Xg, f, pkt);
%             if pdr >= PDR
%                 pdr_flag = pdr_flag/pdrr;
%                 pdr_flag = pdr * pdr_flag;
%             end
        end
        dly_flag = pkt/rate + tau;
        if rPick < 1
            rPick = 1;
        end
        r = loc(rSpace(rPick)); % map to actual location
        cov = Coverage - r;
        d_min = cov * (pkt + tau * RAT_th)/((DEL - dly_flag) * RAT_th);

        if d_min > d_max
            disp('Service Error');
            break;
        end
        
        rCAN = [rCAN; r];
        rPDR = [rPDR; pdr];
        rRAT = [rRAT; rate];
        src = r;
        dist_flag = r + 0.5 * (d_min + d_max);
        PDR = REL/pdr_flag;
        if PDR > PDR_th
            PDR = PDR_th;
        end
        
    end
end
rREL = prod(rPDR);


    

