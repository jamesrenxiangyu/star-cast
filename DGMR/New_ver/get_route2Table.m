function distTable = get_route2Table(rCan, tempLoc)
% rCan: relay candidate idx in dijkstra
% tempLoc: actual cluster information
% distTable: relay candidate information
%           vid, loc_x, loc_y, spd, distance

tempTable = tempLoc(rCan, :);
temp_dist = zeros(size(tempTable, 1),1);
for i = 1:size(tempTable, 1)-1
    dist = norm(tempTable(i+1,2:3) - tempTable(i, 2:3));
    temp_dist(i,1) = dist;
end
distTable = [tempTable, temp_dist];
    
