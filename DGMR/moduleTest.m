%% myRelaySelection.m test module
%% Generate test data
clear; clc;
close all;

Locations = [-1 * randi([0 150], [30, 1]), 4*randi([0, 3], [30, 1])];
Locations = sortrows(Locations, 1, 'descend');
Idx = 10 + (1:size(Locations,1))';
Locations = [Idx, Locations];
Speed = randi([30 40], [30, 1]);
Speed = [Idx, Speed];
refRelay = 37;


%% Network setting
% QoS settings
DEL_th = 20e-3; % E2E delay in sec
SNR_th = 20; % SNR in dB
RAT_th = 20e6; % Data rate in bps
PDR_th = 1 - 1e-4; % Per-hop reliability
REL_th = 1 - 1e-3; % End-to-end reliability

QoS_th = [SNR_th; RAT_th; PDR_th; REL_th; DEL_th];

% Channel settings
Pkt = 1600; % packet size
B = 5e6; % bandwidth in Hz
M = 4; % M-PSK
F = 5e9; % Radio frequency 5GHz
alpha = 3.6; % Path loss component
Xg = 3.7; % Shadowing component in dB
N0 = 2e-9; % Noise power level in mW
Pt_max = 23; % in mW;
Tau = 2e-3; % Delay constant in sec

Ch_par = [M, Pt_max, B, alpha, N0, Xg, F, Pkt, Tau];

% Initial Reference distance
dSNR = distSNR(SNR_th, M, Pt_max, alpha, N0, Xg, F);
dRate = distRate(RAT_th, B, M, Pt_max, alpha, N0, Xg, F);
disRef = min(dSNR, dRate);
covRef = disRef * (DEL_th/(Tau+1e-3));
dDel = covRef * (Pkt + Tau * RAT_th)/(DEL_th * RAT_th);

dmin = dDel;
dmax = min(dSNR, dRate);
if dmin > dmax
    disp('Service Error, adjust QoS');
end
refDis = [dmin, dmax];


%% Test module
[rIdx, rDel, rPdr] = myRelaySelect(REL_th, Locations, Speed, refRelay, Ch_par, refDis)
