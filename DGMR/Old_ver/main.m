clear; clc;
close all;

% Coefficient settings (Fixed)
Dist = (1:0.1:500); % Distance of the Tx range
pt = 10; % Tx power in mW
m = 16; % Modulation
f = 5e9; % Radio frequency in GHz
B = 20e6; % Bandwidth in Hz
alpha = 3.2; % Path loss component
Coverage = 200; % Coverage in m
ErrorRate = 1e-3; % BER thereshold
SNRThreshold = 12; % SNR threshold in dB
Delay = 10; % E2E delay in ms
TxConst = 0.5;
N0 = 1e-9; % Noise power level in mW
X = 1; % Shadowing component
Rate = 50e6; % 50 Mbps
Fading = 1; 
L = 1600 * 8; % Packet length in bit

[PL, SNR, Pb, pb, Pk] = packetLossProb(m, pt, Dist, Rate, f, alpha, N0, f, X, L);
dRef = Dist(find(Pk<1-ErrorRate,1));
[dmin, dmax] = hopDistance(Coverage, SNRThreshold, N0, Delay, f, X, alpha, pt, TxConst);
[dist, rate] = rateDistCalc(B, pt, X, Rate, N0, alpha, Fading, Dist);
% plot(Dist, rate)
% SNR(dB) w.r.t. distance
figure(1)
plot(Dist,SNR);
ylabel('SNR in dB');
xlabel('Distance in meters');
% 
% % Path loss(dB) w.r.t. distance
% figure(2)
% plot(Dist,PL);
% ylabel('PL in dB');
% xlabel('Distance in meters');
% 
% % Link probability w.r.t. distance
% figure(3)
% plot(Dist, Pk);
% xlim([0,30]);
% ylabel('Link probability');
% xlabel('Distance in meters');
% 


                                 
% Coefficient settings (in range)
M = [4, 8, 16, 64]; % Modulation
Pt = (10:2:24); % Tx power in mW
Alpha = (2.7:0.1:3.2); % Path loss component
F = [1, 2, 3, 4, 5].*1e9; % Radio frequency in GHz

%%%% Comparison plots
% figure(1);
% for i = 1:length(M)
% % Path loss model in dB
% PL = 20*log10(f) - 147.55 + 10*alpha*log10(Dist) + X;
% 
% % SNR model in dB
% SNR = 10*log10(pt) - PL - 10*log10(N0);
% 
% % BER model
% Eb_N0 = 0.5 * 10.^(SNR/10);
% k = log2(M(i));
% Pb = 3/(2*k) * erfc(sqrt(0.1*k*Eb_N0));
% 
% % PER model
% Pk = 2^(k-1)/(2^k - 1) * Pb;
% 
% % Link Probability
% P_link = exp(L*log(1-Pk));
% 
% plot(Dist, P_link);
% hold on;
% end
% figure(1);
% title('Comparison over modulation');
% legend('M=4','M=8','M=16','M=64');
% xlim([0,30]);
% 
% figure(2);
% for i = 1:length(Pt)
% % Path loss model in dB
% PL = 20*log10(f) - 147.55 + 10*alpha*log10(Dist) + X;
% 
% % SNR model in dB
% SNR = 10*log10(Pt(i)) - PL - 10*log10(N0);
% 
% % BER model
% Eb_N0 = 0.5 * 10.^(SNR/10);
% k = log2(m);
% Pb = 3/(2*k) * erfc(sqrt(0.1*k*Eb_N0));
% 
% % PER model
% Pk = 2^(k-1)/(2^k - 1) * Pb;
% 
% % Link Probability
% P_link = exp(L*log(1-Pk));
% 
% plot(Dist, P_link);
% hold on;
% end
% figure(2);
% title('Comparison over Tx power');
% xlim([0,30]);
% 
% figure(3);
% for i = 1:length(F)
% m = 16; % Modulation
% % Path loss model in dB
% PL = 20*log10(F(i)) - 147.55 + 10*alpha*log10(Dist) + X;
% 
% % SNR model in dB
% SNR = 10*log10(pt) - PL - 10*log10(N0);
% 
% % BER model
% Eb_N0 = 0.5 * 10.^(SNR/10);
% k = log2(m);
% Pb = 3/(2*k) * erfc(sqrt(0.1*k*Eb_N0));
% 
% % PER model
% Pk = 2^(k-1)/(2^k - 1) * Pb;
% 
% % Link Probability
% P_link = exp(L*log(1-Pk));
% 
% plot(Dist, P_link);
% hold on;
% end
% figure(3);
% title('Comparison over frequency');
% legend();
% xlim([0,100]);
% 
% figure(4);
% for i = 1:length(Alpha)
% % Path loss model in dB
% PL = 20*log10(f) - 147.55 + 10*Alpha(i)*log10(Dist) + X;
% 
% % SNR model in dB
% SNR = 10*log10(pt) - PL - 10*log10(N0);
% 
% % BER model
% Eb_N0 = 0.5 * 10.^(SNR/10);
% k = log2(m);
% Pb = 3/(2*k) * erfc(sqrt(0.1*k*Eb_N0));
% 
% % PER model
% Pk = 2^(k-1)/(2^k - 1) * Pb;
% 
% % Link Probability
% P_link = exp(L*log(1-Pk));
% 
% plot(Dist, P_link);
% hold on;
% end
% figure(4);
% title('Comparison over alpha');
% legend('a=2.7','a=2.8','a=2.9','a=3.0','a=3.1','a=3.2');
% xlim([0,60]);