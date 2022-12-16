%% compute residule link lifetime
function reslinkTime = myLinklife(location, speed, rlocation, rspeed, refdistance)

% Input: location - current vehicle's locations of size 2 x 1
%        speed - current vehicle's speed
%        rlocation - relay vehilce's locations of size 2 x 1
%        rspeed - relay vehicle's speed
%        refdistance - reference distance, dmin and dmax
%
% Output: reslinkTime - residule link lifetime in seconds

dmin = min(refdistance);
dmax = max(refdistance);

dv = rspeed - speed; % speed difference
temp = abs(rlocation - location); % location difference
theta = atan(temp(2)/temp(1)); % angle between two vehicles in terms of pi
if dv > 0
    dref = dmax;
    theta = pi - theta;
elseif dv < 0
    dref = dmin;
else
    fg = round(rand(1));
    if fg == 0
        dref = dmin;
        dv = -1;
    else
        dref = dmax;
        theta = pi-theta;
        dv = 1;
    end
end

%% Compute residual distance
d = norm(temp);
resDist1 = d*cos(theta) + sqrt((dref^2-d^2) + (d*cos(theta))^2);
resDist2 = d*cos(theta) - sqrt((dref^2-d^2) + (d*cos(theta))^2);

resDist = min(abs(resDist1), abs(resDist2));
%% COmpute residual link lifetime
reslinkTime = resDist/abs(dv);




