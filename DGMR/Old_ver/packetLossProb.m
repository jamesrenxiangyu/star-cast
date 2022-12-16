function [PL, SNR, Pb, pb, Pk] = packetLossProb(Modulation, TxPower, Distance, Rate, Bandwidth,...
                                     Alpha, Noise, Frequency, Shadow, PktLength)

Dist = Distance; % Distance of the Tx range
pt = TxPower; % Tx power in mW
m = Modulation; % Modulation
f = Frequency; % Radio frequency in GHz
alpha = Alpha; % Path loss component
N0 = Noise; % Noise power level in mW
X = Shadow; % Shadowing component
L = PktLength; % Packet length in bit
B = Bandwidth;
R = Rate;

% Free space path loss model in dB
FSPL = 20*log10(f) - 147.55;
% Path loss model in dB
PL = FSPL + 10.*alpha*log10(Dist) + X;
% SNR model in dB
SNR = 10*log10(pt) - PL - 10*log10(N0);
% BER model
Eb_N0 = 10.^(SNR/10) * B/f;
k = log2(m);
Pb = 3/(2*k) * erfc(sqrt(0.1*k*Eb_N0));
% PDR = 1-PER model
Pk = (1-Pb).^L;

snr = (0:1:20);
ebn0 = 10.^(snr * 0.1);
% pb = 3/(2*k) * erfc(sqrt(0.1*k*ebn0));
pb = 0.5 * erfc(snr);
end