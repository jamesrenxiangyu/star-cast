function [distance, rate] = rateDistCalc (Bandwidth, Tx_power, Delta, Rate, Noise, alpha, Fading, Dist)

R = Rate;
B = Bandwidth;
Pt = Tx_power;
a = alpha;
g = Delta;
N = Noise;
h = Fading;
d = Dist;

distance = ((2^(R/B) - 1) * (N/(Pt*g*h^2)))^(-1/a);
rate = B * log2(1 + Pt*d.^(-alpha)*h^2*g/N);
