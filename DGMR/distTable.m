% compute the inter-vehilce
function dist_table = distTable(Locations, src_loc)
l = Locations;
num = length(l);
dist_table = zeros(num,1);
src = src_loc(1,1);
for i = 1:num
    dist = l(i) - src;
    dist_table(i) = dist;
    src = l(i);
end
