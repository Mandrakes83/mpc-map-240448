function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

finish = Map_Cell(read_only_vars.discrete_map.goal(1),read_only_vars.discrete_map.goal(2),true);

map_step = read_only_vars.map.discretization_step;

rounded_x = round(public_vars.estimated_pose(1)/map_step)*map_step;
rounded_y = round(public_vars.estimated_pose(2)/map_step)*map_step;

start = Map_Cell(rounded_x,rounded_y,false);

offsets = [
   -1  0;
    0 -1;
    0  1;
    1  0;
   -1  1;
   -1 -1;
    1 -1;
    1  1
];

Prio_Queue = Map_Cell.empty;
Prio_Queue(end+1) = start;

New_map(read_only_vars.discrete_map.dims(1),read_only_vars.discrete_map.dims(2)) = Map_Cell;


while(~isempty(Prio_Queue))

    selected_cell = Prio_Queue(1);

    Prio_Queue(1) = [];
    if(selected_cell == finish)
        break;
    end

    for i = 1:4
    explored_cell = Map_Cell(selected_cell.x+offsets(i,1),selected_cell.y+offsets(i,2),false);

    if(explored_cell.x <= read_only_vars.map.limits(1) || explored_cell.x >= read_only_vars.map.limits(3))
        continue;
    end
    if(explored_cell.y <= read_only_vars.map.limits(2) || explored_cell.y >= read_only_vars.map.limits(4))
        continue;
    end

    idx = find(Prio_Queue == explored_cell, 1); 

    if(isempty(idx))
        explored_cell.HeuresticCost = djikstra_cost(selected_cell);
        explored_cell.Visited = true;
        Prio_Queue(end+1) = explored_cell;
    else
        % duplicity found !LET IT DIE!
    end
    end
    key = [Prio_Queue.HeuresticCost];
    [~, idx] = sort(key);
    Prio_Queue = Prio_Queue(idx);

end

path = [];

end

function [cost] = djikstra_cost(selected)
    movement_cost = 1; %Can be adjusted later on... this is for 4-way search

    cost = selected.HeuresticCost + movement_cost;
end
