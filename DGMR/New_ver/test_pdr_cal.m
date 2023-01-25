%% Test module for linkCal
close all; clear; clc;

%% Network setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600 * 8; % packet size
B = 10e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.6; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = -95; % Background noise in 10 MHz assuming a noise figure of 9dB (dBm)   
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec
beta = 0.06; % beta: traffic density in veh/m. Values tested: 0.06 and 0.12 veh/m
lambda = 10; % lambda: packet transmission frequency in Hz. Values tested: 10 and 25 Hz.

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau, beta, lambda];

%%
Distance = (50:10:120);
[p1 r1] = linkCal(Distance, Ch_par, RAT_th)

pdr = pdr_cal(Ch_par, RAT_th, Distance)

plot(Distance, PDR)
