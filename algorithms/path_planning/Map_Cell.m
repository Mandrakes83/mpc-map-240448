
classdef Map_Cell
    properties
        x       (1,1) double  = 0
        y       (1,1) double  = 0
        Visited (1,1) logical = false
        RealCost(1,1) double = 0
        HeuresticCost(1,1) double = 0
    end

    methods
        function obj = Map_Cell(x, y, visited)
            % Constructor
            if nargin >= 2
                obj.x = x;
                obj.y = y;
            end
            if nargin == 3
                obj.Visited = visited;
            end
        end

        function tf = eq(a, b)
            % Overload ==
            tf = ([a.x] == [b.x]) & ([a.y] == [b.y]);
        end

    end
end
