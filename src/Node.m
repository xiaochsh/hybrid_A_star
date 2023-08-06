classdef Node
    properties
        xidx = 0;
        yidx = 0;
        yawidx = 0;
        D = 0;
        delta = 0;
        x = 0;
        y = 0;
        theta = 0;
        parent = [0,0,0];
        cost = inf;
    end
    methods
        function obj = Node(xidx, yidx, yawidx, D, delta, x, y, theta, parent, cost) 
            obj.xidx = xidx; % pose x index
            obj.yidx = yidx; % pose y index
            obj.yawidx = yawidx; % pose theta index
            obj.D = D; % last path motion resolution
            obj.delta = delta; % last path theta resolution
            obj.x = x;
            obj.y = y;
            obj.theta = theta;
            obj.parent = parent; % [xidx, yidx, yawidx]
            obj.cost = cost;
        end
    end
end