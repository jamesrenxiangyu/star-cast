% Compute the distance threshold by SNR
function dSNR = distSNR(SNR_th, Modulation, TxPower, Alpha, Noise, Shadow, Frequency)
                                 
Pt_max = TxPower; % Tx power in mW
M = Modulation; % Modulation
F = Frequency; % Radio frequency in GHz
alpha = Alpha; % Path loss component
N0 = Noise; % Noise power level in mW
Xg = Shadow; % Shadowing component
D0 = 1000; % Reference distance

% Free space path loss model in dB
lambda = physconst('LightSpeed')/F;
FSPL = fspl(D0,lambda);
% SNR distance in meters
PL = 10*log10(Pt_max) - SNR_th - 10*log10(N0);
dSNR = D0 * (10^((PL - FSPL - Xg) * 0.1 * 1/alpha));
