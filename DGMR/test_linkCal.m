%% Test module for linkCal
close all; clear; clc;

% Channel settings
Pkt = 1600; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 4.2; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = 2e-9; % Noise power level in mW
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau];

distance = (1:1000);
[pdr, rate] = linkCal(distance, Ch_par);

subplot(2,1,1)
plot(distance, pdr)
subplot(2,1,2)
plot(distance, rate)