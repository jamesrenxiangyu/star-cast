clear;clc
%% Initialize the algorithm parameters
epsilon = input('epsilon = ');
numplays = input('number of plays = ');
N = input('number of iterates= ');
Ravg = zeros(numplays,1); % Average reward 
Bavg = zeros(numplays,1); % Average of #Best Action
As = zeros(numplays,1); % Action selected
%% Iterations
    for j=1:N
         m = randn(10,1);
         [~,idx] = max(m);
        [As,R]= Egreedy(numplays,m,epsilon);
        Ravg = Ravg + R;
        As(As ~= idx) = 0; 
        As(As==idx) = 1;   
        Bavg = Bavg + As; 
    end
   %% Plots
    figure(1);
    plot(Ravg./N);
    xlabel('Plays')
    ylabel('reward')
    legend(['ep = ',num2str(epsilon)]);
    figure(2);
    plot(Bavg./N);
    xlabel('Plays')
    ylabel('% Optimal action')
    legend(['ep = ',num2str(epsilon)]);