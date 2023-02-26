% Compute the distance threshold by data rate 
function dist = get_RateDist(QoS_th, Ch_par, flag_dens)
Rate_th = QoS_th(2);
B = Ch_par(3);

% Data rate
SNR = 2^(Rate_th/B) - 1;
SNR_db = 10*log10(SNR);
QoS_th(1) = SNR_db;
dist = get_SNRDist(QoS_th, Ch_par, flag_dens);
