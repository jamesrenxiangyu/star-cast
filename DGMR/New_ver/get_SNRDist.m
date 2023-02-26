% Compute the distance threshold by SNR
function dist = get_SNRDist(QoS_th, Ch_par, flag_dens)
SNR_th = QoS_th(1);
Noise = Ch_par(5);
Frequency = Ch_par(7);
Pt_max = Ch_par(2);

Pt_dBm = 10 * log10(Pt_max);

if flag_dens == 1
    % Sparse network
    X = 7.56 - 2.7 * log10(Frequency/1e9);
    dist = 10.^(0.025 * (Pt_dBm - SNR_th - Noise - X));
elseif flag_dens == 0
    % Dense network
    X = 27 + 20 * log10(Frequency/1e9);
    dist = 10.^((1/22.7) * (Pt_dBm - SNR_th - Noise - X));
else
    disp('Error! Identify network density')
end

