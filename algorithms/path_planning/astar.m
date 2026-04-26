function path = astar(read_only_vars, public_vars)

map_step = read_only_vars.map.discretization_step;

occMap = read_only_vars.discrete_map.map';
distToObstacle = obstacleDistanceBFS(occMap);

mapW   = read_only_vars.discrete_map.dims(1);
mapH   = read_only_vars.discrete_map.dims(2);

% Start (grid indices)

start_x = floor(public_vars.estimated_pose(1) / map_step) + 1;
start_y = floor(public_vars.estimated_pose(2) / map_step) + 1;

% Clamp to map bounds
start_x = max(1, min(start_x, mapW));
start_y = max(1, min(start_y, mapH));

% Goal (grid indices)
goal_x = read_only_vars.discrete_map.goal(1);
goal_y = read_only_vars.discrete_map.goal(2);

% --- 4-way Dijkstra heuristic (obstacle-aware) ---
dijkstraMap = computeDijkstra4(occMap, mapW, mapH, goal_x, goal_y);

start = Map_Cell(start_x, start_y);
goal  = Map_Cell(goal_x, goal_y);

start.gCost = 0;
start.hCost = dijkstraMap(start_x, start_y);
start.fCost = start.gCost + start.hCost;

% 8-connected neighbors: [dx dy cost]
neighbors = [
    -1  0  1;
     1  0  1;
     0 -1  1;
     0  1  1;
    -1 -1  sqrt(2);
    -1  1  sqrt(2);
     1 -1  sqrt(2);
     1  1  sqrt(2)
];

openSet = start;
allCells = containers.Map("KeyType","char","ValueType","any");
allCells(key(start)) = start;

while ~isempty(openSet)

    % Pick node with minimum fCost
    [~, idx] = min([openSet.fCost]);
    current = openSet(idx);
    openSet(idx) = [];

    if current == goal
        goal = current;
        break;
    end

    current.Closed = true;

    for i = 1:size(neighbors,1)

        dx = neighbors(i,1);
        dy = neighbors(i,2);
        moveCost = neighbors(i,3);

        nx = current.x + dx;
        ny = current.y + dy;

        % Bounds check
        if nx < 1 || ny < 1 || nx > mapW || ny > mapH
            continue;
        end

        % Occupancy check
        if occMap(nx, ny) == 1
            continue;
        end

        % Prevent diagonal corner-cutting
        if dx ~= 0 && dy ~= 0
            if occMap(current.x + dx, current.y) == 1 || ...
               occMap(current.x, current.y + dy) == 1
                continue;
            end
        end

        k = sprintf('%d_%d', nx, ny);
        if isKey(allCells, k)
            neighbor = allCells(k);
        else
            neighbor = Map_Cell(nx, ny);
            allCells(k) = neighbor;
        end

        if neighbor.Closed
            continue;
        end

        
        desired_clearance = 3;    % safety radius (cells)
        wall_weight = 10;
        
        if distToObstacle(nx,ny) <= desired_clearance
            penalty = wall_weight * (desired_clearance + 1 - distToObstacle(nx,ny));
        else
            penalty = 0;
        end

        tentative_g = current.gCost + moveCost + penalty;


        if tentative_g < neighbor.gCost
            neighbor.Parent = current;
            neighbor.gCost  = tentative_g;
            neighbor.hCost  = dijkstraMap(nx, ny);
            neighbor.fCost  = neighbor.gCost + neighbor.hCost;

            if ~any(openSet == neighbor)
                openSet(end+1) = neighbor;
            end
        end
    end
end

% --- Reconstruct path (world coordinates) ---
path = [];
node = goal;

while ~isempty(node)
    path(end+1,:) = [ ...
        (node.x - 0.5) * map_step, ...
        (node.y - 0.5) * map_step ];
    node = node.Parent;
end

path = flipud(path);

end

function D = computeDijkstra4(occMap, W, H, gx, gy)

D = inf(W, H);

% Goal must be free
if occMap(gx, gy) == 1
    return;
end

D(gx, gy) = 0;
queue = [gx gy];

offsets = [
    -1  0;
     1  0;
     0 -1;
     0  1
];

while ~isempty(queue)
    current = queue(1,:);
    queue(1,:) = [];

    for i = 1:4
        nx = current(1) + offsets(i,1);
        ny = current(2) + offsets(i,2);

        if nx < 1 || ny < 1 || nx > W || ny > H
            continue;
        end

        if occMap(nx, ny) == 1
            continue;
        end

        newCost = D(current(1), current(2)) + 1;
        if newCost < D(nx, ny)
            D(nx, ny) = newCost;
            queue(end+1,:) = [nx ny];
        end
    end
end

end

function k = key(cell)
    k = sprintf('%d_%d', cell.x, cell.y);
end

function distToObstacle = obstacleDistanceBFS(rawMap)
% rawMap: 0 = free, 1 = occupied
% Pure MATLAB, NO TOOLBOX

[W, H] = size(rawMap);
distToObstacle = inf(W, H);

queue = zeros(W*H, 2);
qs = 1; qe = 0;

% Initialize from obstacles
for x = 1:W
    for y = 1:H
        if rawMap(x,y) == 1
            distToObstacle(x,y) = 0;
            qe = qe + 1;
            queue(qe,:) = [x y];
        end
    end
end

offsets = [-1 0; 1 0; 0 -1; 0 1];

while qs <= qe
    cx = queue(qs,1);
    cy = queue(qs,2);
    qs = qs + 1;

    for i = 1:4
        nx = cx + offsets(i,1);
        ny = cy + offsets(i,2);

        if nx<1 || ny<1 || nx>W || ny>H
            continue;
        end

        if distToObstacle(nx,ny) > distToObstacle(cx,cy) + 1
            distToObstacle(nx,ny) = distToObstacle(cx,cy) + 1;
            qe = qe + 1;
            queue(qe,:) = [nx ny];
        end
    end
end
end