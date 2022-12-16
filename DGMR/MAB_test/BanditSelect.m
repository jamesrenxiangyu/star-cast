function [cR,cQ]=BanditSelect(Q,meu,epsilon)
    % Q : estimated reward (Qt for all the bandits)
    % meu : the real reward for all the bandits
    % cR: estmated reward with noise
    % cQ: bandit selection
    numbandits = length(Q);
    rnd = rand(1);
 if rnd < epsilon % exploration!
        cQ = ceil(rand*numbandits);
        cR = randn + meu(cQ);
 else % exploitation!
        idx = find(Q==max(Q));
        rnd2 = rand(1);
        m = ceil(rnd2 * length(idx));
        cQ = idx(m);
        cR = randn + meu(cQ);
 end
end