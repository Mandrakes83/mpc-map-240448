classdef Map_Cell < handle
    properties
        x (1,1) double          % grid index X
        y (1,1) double          % grid index Y

        gCost (1,1) double = inf   % cost from start
        hCost (1,1) double = 0     % heuristic to goal
        fCost (1,1) double = inf   % g + h

        Parent Map_Cell = Map_Cell.empty

        Closed (1,1) logical = false
    end

    methods
        function obj = Map_Cell(x, y)
            if nargin >= 2
                obj.x = x;
                obj.y = y;
            end
        end

        function tf = eq(a, b)
            tf = ([a.x] == [b.x]) & ([a.y] == [b.y]);
        end
    end
end