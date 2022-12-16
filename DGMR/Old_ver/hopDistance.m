%%% Key parameters in terms of hop and link probability
function [dmin, dmax] = hopDistance(Coverage, SNRThreshold, Noise,...
                                    Delay, Frequency, Shadow, Alpha, TxPower, TxConst)
c = Coverage;
thS = SNRThreshold;
T = Delay;
f = Frequency;
X = Shadow;
a = Alpha;
pt = TxPower;
t = TxConst; % per-hop delay constant in ms
N0 = Noise;
% minimum per-hop distance
dmin = c/(T/t);
% maximum per-hop distance using SNR
PL = 10*log10(pt) - thS - 10*log10(N0);
dmax_exp = 1/(10*a) * (PL+147.55-20*log10(f)-X);
dmax = 10^dmax_exp;
end