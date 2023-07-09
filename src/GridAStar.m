function costMap = GridAStar(obstList, goal, gres)
    [minx, miny, obmap] = CalcObstMap(obstList, gres);
    col = ceil((goal(1) - minx) / gres);
    row = ceil((goal(2) - miny) / gres);
    goal = [row, col];
    costMap = zeros(size(obmap), 'single'); 
    dim = size(obmap);     
    for i = 1 : dim(1)
        for j = 1 : dim(2)
            if obmap(i, j) == 1
                costMap(i,j) = single(1e9);
                continue
            elseif i == row && j == col
                continue
            end
            start = [i, j];
            cost = AStarSearch(start, goal, obmap);
            costMap(i, j) = cost;
        end
    end    
end

function cost = AStarSearch(start, goal, obmap)
    dim = size(obmap);
    Grids = zeros(dim(1), dim(2), 4, 'single');
    for i = 1 : dim(1)
        for j = 1 : dim(2)
            Grids(i, j, 1) = i; % x of parent pos
            Grids(i, j, 2) = j; % y of parent pos
            Grids(i, j, 3) = norm(([i, j] - goal)); % precost
            Grids(i, j, 4) = single(1e9); % postcost
        end
    end
    Open = repmat([0, 0], ceil(dim(1) * dim(2) / 2), 1);
    u16OpenIdx = uint16(1);
    Close = Open;
    u16CloseIdx = uint16(0);
    Open(u16OpenIdx, :) = start;
    Grids(start(1), start(2), 4) = single(0);
    while u16OpenIdx > uint16(0)
        [wknode, Open, u16OpenIdx] = PopOpen(Open, Grids, u16OpenIdx);
        [Grids, Open, Close, u16OpenIdx, u16CloseIdx] = Update(wknode, goal, obmap, Grids, Open, Close, u16OpenIdx, u16CloseIdx);
        Close(u16CloseIdx, :) = wknode;
    end
    cost = Grids(goal(1), goal(2), 3) + Grids(goal(1), goal(2), 4);
end

function [Grids, Open, Close, u16OpenIdx, u16CloseIdx] = Update(wknode, goal, obmap, Grids, Open, Close, u16OpenIdx, u16CloseIdx)
    dim = size(obmap);
    for i = -1 : 1
        for j = -1 : 1
            adjnode = wknode + [i, j];
            row = adjnode(1);
            col = adjnode(2);
            if i == 0 && j == 0
                continue
            elseif row < 1 || row > dim(1)
                continue
            elseif col < 1 || col > dim(2)
                continue
            elseif obmap(row, col) == 1
                continue
            end
            tcost = Grids(wknode(1), wknode(2), 4) + norm([i, j]);
            if Grids(row, col, 4) > tcost
                Grids(row, col, 1) = wknode(1);
                Grids(row, col, 2) = wknode(2);
                Grids(row, col, 4) = tcost;
                % add adjnode to Open except wknode is goal
                bUpdateOpen = true;
                for k = 1 : u16OpenIdx
                    if Open(k, 1) == adjnode(1) && Open(k, 2) == adjnode(2) || ...
                            goal(1) == adjnode(1) && goal(2) == adjnode(2)
                        bUpdateOpen = false;
                        break;
                    end
                end
                if bUpdateOpen
                    u16OpenIdx = u16OpenIdx + uint16(1);
                    Open(u16OpenIdx, :) = adjnode;
                end
                % if adjnode is in Close remove it
                u16RemoveIdx = uint16(0);
                for k = 1 : u16CloseIdx
                   if Close(k, 1) == adjnode(1) && Close(k, 2) == adjnode(2) && ...
                           tcost < Grids(adjnode(1), adjnode(2), 4)
                       u16RemoveIdx = uint16(k);
                   end
                end
                if u16RemoveIdx > uint16(0)
                    for k = u16RemoveIdx + 1 : u16CloseIdx
                        Close(k - 1, :) = Close(k, :);
                    end
                    u16CloseIdx = u16CloseIdx - uint16(1);
                end
            end
        end
    end
    u16CloseIdx = u16CloseIdx + uint16(1);
end

function [wknode, Open, u16OpenIdx] = PopOpen(Open, Grids, u16OpenIdx)
    mincost = single(1e9);
    minidx = 1;
    for i = 1 : u16OpenIdx
        node = Open(i, :);
        tcost = Grids(node(1), node(2), 3) + Grids(node(1), node(2), 4);
        if tcost < mincost
            minidx = i;
            mincost = tcost;
        end
    end
    wknode = Open(minidx, :);
    for i = minidx + 1 : u16OpenIdx
        Open(i - 1, :) = Open(i, :);
    end
    u16OpenIdx = u16OpenIdx - uint16(1);
end

function [minx, miny, obmap] = CalcObstMap(obstlist, gres)
    minx = min(obstlist(:, 1));
    maxx = max(obstlist(:, 1));
    miny = min(obstlist(:, 2));
    maxy = max(obstlist(:, 2));
    xwidth = maxx - minx;
    xwidth = ceil(xwidth / gres);
    ywidth = maxy - miny;
    ywidth = ceil(ywidth / gres);
    obmap = zeros(ywidth, xwidth);
    for i = 1 : ywidth
        for j = 1 : xwidth
            ix = minx+(j - 1 / 2) * gres;
            iy = miny+(i - 1 / 2) * gres;
            [~, D] = knnsearch(obstlist, [ix, iy]);
            if D < 0.708 * gres
                obmap(i, j) = 1;
            end
        end
    end
end