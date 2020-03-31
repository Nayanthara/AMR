function[feasible] = checkFeasibility(pose,map)

feasible = 0;
N = size(map,1);

% Check whether proposed pose is within map
outerPolygon = [min(map(:,[1 3]),[],'all') min(map(:,[2 4]),[],'all');...
    min(map(:,[1 3]),[],'all') max(map(:,[2 4]),[],'all');...
    max(map(:,[1 3]),[],'all') max(map(:,[2 4]),[],'all');...
    max(map(:,[1 3]),[],'all') min(map(:,[2 4]),[],'all');...
    min(map(:,[1 3]),[],'all') min(map(:,[2 4]),[],'all')];
[in,on] = inpolygon(pose(1),pose(2),outerPolygon(:,1),outerPolygon(:,2));
if in(1)& ~on(1)
    feasible = 1;
end
end
