% Compute the data rate achieved for a given distance
function [PDR, Rate, mySNR]= get_linkCal(Distance, Ch_par, RAT_th)
                             
% linkCal is the main script of the implementation of the analytical 
% models of the communication performance of IEEE 802.11p described in the following paper:
 % Configuration parameters and settings:
    step_dB = 0.1;                  % Discrete steps to compute the PDF of the SNR and SINR (dB) 
    distance_tx_to_rx = Distance; % Tx-Rx distances to evaluate (m)
    Pt = Ch_par(2);
    BW = Ch_par(3);                      % Channel bandwidth (Hz)    
    Psen = -85;                     % Sensing threshold (dBm)    
    noise = Ch_par(5);                    % Background noise in 10 MHz assuming a noise figure of 9dB (dBm)   
    sigma = 13e-6;                  % aSlotTime in 802.11-2012 (seconds)
    frequency = Ch_par(7);          % Carrier frequency in Hz
    Pkt = Ch_par(8);                         % length of headers in Omnet++ approx (Bytes)
    beta = Ch_par(10);              % beta: traffic density in veh/m. 
    lambda = Ch_par(11);            % lambda: packet transmission frequency in Hz
    Rd = RAT_th;
    
    PathLoss = get_PL(Distance, frequency);
    mySNR = 10 .* log10(Pt) - PathLoss - noise;
    
    Rate = BW .* log2(1 + 10.^(mySNR./10));
    
    Ttr = 40e-6 + Pkt*8/Rd;       % Packet transmission time (duration) in 802.11-2012 (page 1588,1591) =  T_preamble + T_signal + T_data ; t_preamble = 32 us , t_signal = 8 us , t_data = nºbits/data_rate

    d_aux=-1500:1500;
    [ PL, std_dev ] = get_PL(d_aux, frequency);    % Pathloss and shadowing standard deviation for the propagation model considered
    
    
    % Compute the PSR (Packet Sensing Ratio):
    PSR = 0.5 * ( 1 + erf( ( Pt - PL - Psen)./( std_dev*sqrt(2) ) ) );   % Equation (13)    
    
    % Compute the CBR (Channel Busy Ratio):   
    CBR_u = beta * lambda * Ttr * sum(PSR);           % Equation (34)
    CBR = - 0.2481*CBR_u^2 + 0.913*CBR_u + 0.003844;  % Equation (35)

    % Compute SEN and PRO errors due to propagation effects for all
    % distances between tx and rx:
    
    deltaSEN_pre = zeros(1,length(distance_tx_to_rx));  % Initialization
    deltaPRO_pre = zeros(1,length(distance_tx_to_rx));  % Initialization
    
    for i=1:length(distance_tx_to_rx)
        
        % Compute the probability of error due to a received signal power
        % below the sensing power threshold for a distance
        % distance_tx_to_rx(i) between tx and rx:
        
        [PL_Tx_Rx(i) std_dev_Tx_Rx(i)] = get_PL( distance_tx_to_rx(i), frequency);                           % Pathloss and shadowing standard deviation for the propagation model considered
        deltaSEN_pre(i) = 0.5 * (1 - erf( ( Pt - PL_Tx_Rx(i) - Psen )/(std_dev_Tx_Rx(i)*sqrt(2)) ) ); % Equation (12)
            
        % Compute the probability of error due to a insufficient SNR for a distance
        % distance_tx_to_rx(i) between tx and rx:

        [SNR PDF_SNR] = get_SINRdistribution( Pt-PL_Tx_Rx(i) , -inf , std_dev_Tx_Rx(i) , std_dev_Tx_Rx(i) , noise , Psen , step_dB);   % Distribution of the SNR of the received packet (without interference, i.e. -inf dB) 
        Eb_No = SNR + 10*log10(BW/Rd);   % Linear transformation
        PDF_Eb_No = PDF_SNR;             % SNR and Eb/No have the same probability distribution
        deltaPRO_pre(i) = get_FER( Eb_No , PDF_Eb_No , step_dB );    % Equation (24)
                    
    end

    % Compute the probability of error because the receiver is busy and the 
    % probability of error due to collision for all distances between tx and rx:
    
    Lint_max = round(500*beta)/beta;         % Distance to the farthest interfering vehicle. Up to 500m distances considered to speed up the calculations.
    distance_int_to_rx = [-Lint_max : 1/beta : Lint_max];           % Distances from all the interfering vehicles to the receiving vehicle.
    distance_int_to_rx ( (length(distance_int_to_rx)+1)/2) = [];    % Remove from the list the position of the receiving vehicle. 
    
    R_PSR = xcorr(PSR);  % Autocorrelation of the PSR function. Equation (20)
    R_PSR = R_PSR(2*max(d_aux)+1:end) / max(R_PSR);  % Remove left part of the function and normalize
    
    for d=1:length(distance_tx_to_rx)

        distance_int_to_tx = distance_int_to_rx + distance_tx_to_rx(d);   % Distances from all the interfering vehicles to the transmitting vehicle.
        
        for i = 1:length(distance_int_to_rx)  % Compute the probability for every interfering vehicle

            [PL_i_Rx std_dev_i_Rx] = get_PL( abs(distance_int_to_rx(i)), frequency);  % Pathloss and shadowing for interf and rx
            [PL_i_Tx std_dev_i_Tx] = get_PL( abs(distance_int_to_tx(i)), frequency);  % Pathloss and shadowing for interf and tx

            if deltaPRO_pre(d) == 1                
                p_int(i) = 0;     % If the probability of propagation error is one, we don't need to calculate the collision error because it is zero.               
            else                
                [SINR PDF_SINR] = get_SINRdistribution( Pt-PL_Tx_Rx(d) , Pt-PL_i_Rx , std_dev_Tx_Rx(d) , std_dev_i_Rx , noise , Psen , step_dB);   % % Distribution of the SINR of the received packet considering vehicle vi as interferer
                Eb_No = SINR + 10*log10(BW/Rd);  % Convert SINR to Eb/No
                PDF_Eb_No = PDF_SINR;            % Convert SINR to Eb/No
                pSINR(i) = get_FER( Eb_No , PDF_Eb_No , step_dB );              % Equation (30)
                p_int(i) = (pSINR(i)-deltaPRO_pre(d)) / (1 - deltaPRO_pre(d));  % Equation (31)
            end
            
            p_DET_i_Rx(i) = 0.5 * (1 + erf(( Pt - PL_i_Rx - Psen )/(sqrt(2)*std_dev_i_Rx)));  % Probability that the receiving vehicle (vr) senses the interfering one (vi) 
            p_DET_i_Tx(i) = 0.5 * (1 + erf(( Pt - PL_i_Tx - Psen )/(sqrt(2)*std_dev_i_Tx)));  % Probability that the interfering vehicle (vi) senses the transmitting one (vt) or viceversa
            
            p_sim_CT(i) = sigma  * lambda *  p_DET_i_Tx(i)   / (1-CBR*R_PSR( round( abs(distance_int_to_tx(i)) ) + 1 )) ; % Equation (22)
            
%             % Differentiate when the interfering is closer than the transmitter: Equation (21)
%             if abs(distance_int_to_rx(i)) < abs(distance_tx_to_rx(d))                 
%                 p_RXB_CT(i) = p_sim_CT(i) * p_DET_i_Rx(i) ;  
%             else                
%                 p_RXB_CT(i) =  0 ;                
%             end
%             
%             p_RXB_HT(i) = Ttr * lambda * p_DET_i_Rx(i) *(1-p_DET_i_Tx(i)) / (1-CBR*R_PSR( round( abs(distance_int_to_tx(i)) ) + 1 ));  % Equation (16)
%             
            % Differentiate when the interfering is closer than the transmitter: Equation (32)
            if abs(distance_int_to_rx(i)) >= abs(distance_tx_to_rx(d)) 
                p_COL_CT(i) = p_int(i) * p_sim_CT(i); 
            else
                p_COL_CT(i) = 0; 
            end
            
            p_sim_HT(i) = Ttr * lambda * (1-p_DET_i_Tx(i)) / (1-CBR*R_PSR( round( abs(distance_int_to_tx(i)) ) + 1 )) ; % Equation (18)
            
            p_COL_HT(i) = p_int(i) * p_sim_HT(i) + p_int(i) * p_sim_HT(i) * ( 1-p_DET_i_Rx(i) ) ;  % Equation (28)
             
        end

        % Combine the probability for all potential interfering vehicles to compute the overall probabilities:
%         deltaRXB_pre(d) = 1 - prod( 1 - ( p_RXB_CT + p_RXB_HT ) );   % Equation (14) 
        deltaCOL_pre(d) = 1 - prod( 1 - ( p_COL_CT + p_COL_HT ) ) ;  % Equation (26)

        % Calculate final probabilities for each type of error:
        deltaSEN(d) = deltaSEN_pre(d);                                                          % Equation (3)
%         deltaRXB(d) = deltaRXB_pre(d) * ( 1 - deltaSEN_pre(d) );                                % Equation (4)
        deltaPRO(d) = deltaPRO_pre(d) * ( 1 - deltaSEN_pre(d) );      % Equation (5)
        deltaCOL(d) = deltaCOL_pre(d) * ( 1 - deltaSEN_pre(d) ) * ( 1 - deltaPRO_pre(d) );    % Equation (6)

    end
    % Compute the Packet Delivery Ratio:    
%     PDR = 1 - deltaSEN - deltaPRO - deltaCOL; % Equation (1)
    PDR = 1 - deltaSEN - deltaPRO; % Equation (1)
    PDR(PDR >= 0.9999) = 0.9999;
    