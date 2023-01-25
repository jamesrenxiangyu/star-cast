% Compute the data rate achieved for a given distance
function [PDR, Rate]= linkCal(Distance, Ch_par)
                             
L = Distance; % Distance of the Tx range
M = Ch_par(1); % Modulation
Pt_max = Ch_par(2); % Tx power in mW
B = Ch_par(3); % Channel bandwidth
alpha = Ch_par(4); % Path loss component
N0 = Ch_par(5); % Noise power level in mW
Xg = Ch_par(6); % Shadowing component
F = Ch_par(7); % Radio frequency in GHz
Pkt = Ch_par(8); % Packet length in bit



D0 = 1000; % Reference distance

% Free space path loss model in dB
lambda = physconst('LightSpeed')/F;
FSPL = fspl(D0,lambda);
% Path loss model in dB
PL = FSPL + 10*alpha*log10(L./D0) + Xg;
% Received power
power = 10.^(0.1*(10*log10(Pt_max) - PL));
% SNR model in dB
SNR_db = 10*log10(Pt_max) - PL - 10*log10(N0);
SNR = 10.^(0.1*SNR_db);
% Data rate
Rate = B * log2(1 + SNR);
% BER using M-PSK 
% ber = bercoding(ebn0,'Hamming','hard', 15);
k = log2(M);
% Eb/N0 
ebn0 = SNR .* (B./Rate);
% pe = 0.5 * erfc(ebn0);
% pe = erfc(sqrt(2*ebn0)*sin(pi/M));
pe = erfc(sqrt(2*ebn0));
BER = 2^(k-1)/(2^k -1 ) * pe;
% PDR
PDR = (1-BER).^Pkt;