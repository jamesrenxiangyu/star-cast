function [As,R]= Egreedy(N,meu,epsilon)
% N = number of plays
% meu = actual reward for each bandit
% As= action selected on play j, j=1:N
% Q = reward estimate for each bandit
% R = an N*1 vector , the element j of the vector contains the reward at play number j

numbandits=length(meu);
ActNum = zeros(numbandits,1); % 
ActVal = zeros(numbandits,1);
Q = zeros(numbandits,1);
As = zeros(N,1);
R = zeros(N,1);

for j=1:N
   [cR,cQ]=BanditSelect(Q,meu,epsilon); % choosing the bandit in accordance to the epsilon greedy algorithm
    R(j) = cR;
    As(j) = cQ;
    % update the estimated mean reward for the selected bandit
    ActNum(cQ)=ActNum(cQ) + 1;
    ActVal(cQ) = ActVal(cQ) + cR;
    Q(cQ) = ActVal(cQ)/ActNum(cQ);
end                                                                                                                                        