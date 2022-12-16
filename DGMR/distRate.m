% Compute the distance threshold by data rate 
function dRate = distRate(Rate_th,  Bandwidth, Modulation, TxPower, Alpha, Noise, Shadow, Frequency)
B = Bandwidth;
Pt_max = TxPower; % Tx power in mW
M = Modulation; % Modulation
F = Frequency; % Radio frequency in GHz
alpha = Alpha; % Path loss component
N0 = Noise; % Noise power level in mW
Xg = Shadow; % Shadowing component

% Data rate
SNR = 2^(Rate_th/B) - 1;
SNR_db = 10*log10(SNR);
dRate = distSNR(SNR_db, M, Pt_max, alpha, N0, Xg, F);
