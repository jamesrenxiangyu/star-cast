%BPSK BER
clear; close all;
const=[1 -1];
size=100000;
iter_max=1000;
EbN0_min=0;
EbN0_max=10;
SNR=[];BER=[];
for EbN0 = EbN0_min:EbN0_max
    EbN0_lin=10.^(0.1*EbN0);
    noise_var=0.5/(EbN0_lin); % s^2=N0/2
    iter = 0;
    err = 0;
    while (iter <iter_max && err <100)
        bits=randsrc(1,size,[0 1]);
        s=const(bits+1);
        x = s + sqrt(noise_var)*randn(1,size);
        bit_hat=(-sign(x)+1)/2;
        err = err + sum(bits ~= bit_hat);
        iter = iter + 1;
    end
    SNR =[SNR EbN0];
    BER = [BER err/(size*iter)];
end
semilogy(SNR,BER);grid;
xlabel('E_bN_0');
ylabel('BER');
title('BPSK over AWGN channel');