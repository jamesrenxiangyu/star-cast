function [state, observ] = myKF(loc_x, loc_y, speed, angle, t)

state_old = [loc_x; loc_y; speed; angle];


% State transition matrix
F = [1, 0, t*cos(angle), 0; 0 1 t*sin(angle) 0; 0 0 1 0; 0 0 0 1];

% System uncertainty matrix
a = normrnd(0, 1); % for speed
w = normrnd(0, (pi/60)^2); % for angle
W = [0.5*t^2*cos(angle); 0.5*t^2*sin(angle); t*a; t*w];

% System uncertainty covariance
Q = W*W';

% Observation matrix
H = eye(4);

% Measurement noise matrix
nx = 1e-2;
ny = 1e-2;
nv = 1e-2;
na = 1e-2;
noise = [nx^2, ny^2, nv^2, na^2];
R = diag(noise);

% States and Observation
observ_old = H * state_old + mvnrnd(zeros(4,1), R)';

% Initialize Priori covariance matrix
Pp = (observ_old - state_old) * (observ_old - state_old)';

% Update
state_pre = F *  state_old;
observ_new = H * state_pre + mvnrnd(zeros(4,1), R)';
P = F * Pp * F' + Q;
K = P * H' * inv(H * P * H' + R);
state_new = state_pre + K * (observ_new - H * state_pre);

state = state_new;
observ = observ_new;






