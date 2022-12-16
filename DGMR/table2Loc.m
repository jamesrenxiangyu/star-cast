function dLoc = table2Loc(dTable)
Num = size(dTable,1);
dLoc = zeros(Num + 1, 1);
for i = 2:Num+1
    dLoc(i) = sum(dTable(1:i-1));
end