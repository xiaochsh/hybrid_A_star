function [x, y, th] = HybridAStar(startPose, endPose, veh, cfg)
    mres = cfg.MOTION_RESOLUTION; % motino resolution 
    
    % 把起始的位姿(x, y, theta)转换为grid上的栅格索引
    [isok, xidx, yidx, thidx] = CalcIdx(startPose(1), startPose(2), startPose(3), cfg);
    if isok % 把位姿栅格定义为一个结点，形成链表结构
        tnode = Node(xidx, yidx, thidx, mres, 0, startPose(1), startPose(2), startPose(3), [xidx, yidx, thidx], 0);
    end
    Open = repmat(Node(0, 0, 0, 0, 0, 0, 0, 0, [0, 0, 0], 0), 1000, 1);
    u16OpenIdx = uint16(1);
    Open(u16OpenIdx) = tnode;
    Close = Open;
    u16CloseIdx = uint16(0);

    % init
    x = zeros(0, 1000, 1, 'single');
    y = zeros(0, 1000, 1, 'single');
    th = zeros(0, 1000, 1, 'single');
    
    iter = uint8(0);
    while u16OpenIdx > uint16(0)
        % pop the least cost node from open to close
        [wknode, Open, u16OpenIdx] = PopNode(Open, u16OpenIdx, cfg);
        [isok, idx] = inNodes(wknode, Close, u16CloseIdx);
        
        % 判断是否在Close集合内，若是则将wknode移植集合末尾
        if isok
            for i = idx + 1 : u16CloseIdx
                Close(i - 1) = Close(i);
            end
            Close(u16CloseIdx) = wknode;
        else
            u16CloseIdx = u16CloseIdx + uint16(1);
            Close(u16CloseIdx) = wknode;
        end
        
        if rem(iter, 5) == uint8(0)
            % 以wknode为起点生成到终点的无碰撞RS曲线
            [isready, path] = AnalysticExpantion([wknode.x, wknode.y, wknode.theta], endPose, veh, cfg);
        else
            pvec = [endPose(1) - wknode.x; endPose(2) - wknode.y];
            phi = endPose(3) - wknode.theta;
            if norm(pvec) < mres && abs(phi) < single(pi / 60)
                isready = true;
            else
                isready = false;
            end
        end
        if  isready
            [x, y, th] = getFinalPath(path, Close, u16CloseIdx, veh, cfg);
            break % 如果能直接生成无碰撞RS曲线RS曲线，则跳出while循环
        end
        [Open, u16OpenIdx, Close] = Update(wknode, Open, u16OpenIdx, Close, u16CloseIdx, veh, cfg); % 使用
        iter = iter + uint8(1);
    end
end

function [x, y, th] = getFinalPath(path, Close, u16CloseIdx, veh, cfg)
    wknode = Close(u16CloseIdx); % Close集合最后节点为无碰撞RS曲线起点
    u16CloseIdx = u16CloseIdx - uint16(1);
    nodes = repmat(Node(0, 0, 0, 0, 0, 0, 0, 0, [0, 0, 0], 0), 1000, 1);
    nodesNum = uint16(1);
    nodes(nodesNum) = wknode;
    % 找目标点wknode的parent, 回溯，直到Close集合为空
    while u16CloseIdx > uint16(0)
        parent = wknode.parent;
        % 计算从目标返回到起始点的路径点序列，放入nodes中
        for i = u16CloseIdx : -1 : 1
            tnode = Close(i);
            if tnode.xidx == parent(1) && tnode.yidx == parent(2) && tnode.yawidx == parent(3)
                nodesNum = nodesNum + uint16(1);
                nodes(nodesNum) = tnode;
                wknode = tnode;
                parentIdx = i;
                break
            end
        end
        for i = parentIdx + 1 : u16CloseIdx
            Close(i - 1) = Close(i);
        end
        u16CloseIdx = u16CloseIdx - uint16(1);
    end
    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    
    x = zeros(0, 1000, 1, 'single');
    y = zeros(0, 1000, 1, 'single');
    th = zeros(0, 1000, 1, 'single');
    idx = uint16(0);
    
    % 最终的路径：1-只有A*路径；2-只有RS曲线路径；3-混合A*路径
    % 路径要么是纯RS路径，要么是由RS路径和混合A*组合一起来的路径，先处理混合A*的结点，最后处理RS路径，肯定有RS路径
    if nodesNum > uint16(1)
        % nodes中第一个点作为RS曲线的起点，其余点位混合A*路径
        for i = nodesNum: -1 : 2
            tnode = nodes(i);
            ttnode = nodes(i - 1);

            px = tnode.x;
            py = tnode.y;
            pth = tnode.theta;
            idx = idx + uint16(1);
            x(idx) = px;
            y(idx) = py;
            th(idx) = pth;

            nlist = floor(cfg.MIN_PATH_LENGTH / cfg.MOTION_RESOLUTION); 
            for j = 1 : nlist
                [px, py, pth] = VehicleDynamic(px, py, pth, ttnode.D, ttnode.delta, veh.WB);
                idx = idx + uint16(1);
                x(idx) = px;
                y(idx) = py;
                th(idx) = pth;
            end
        end
    else
        % 最后一个结点是纯的RS路径终点
        tnode = nodes(1);
        idx = idx + uint16(1);
        x(idx) = tnode.x;
        y(idx) = tnode.y;
        th(idx) = tnode.theta;
    end
    
    % RS曲线的起点不加入轨迹
    px = nodes(1).x;
    py = nodes(1).y;
    pth = nodes(1).theta;

    types = path.type;
    t = rmin * path.t;
    u = rmin * path.u;
    v = rmin * path.v;
    w = rmin * path.w;
    segs = [t, u, v, w, rmin * path.x]; % avoid duplicate of x
    for i = 1 : 5
        if abs(segs(i)) < single(1e-3)
            continue
        end
        s = sign(segs(i)); % 前进或后退
        if types(i) == 'S'          
            tdelta = 0;
        elseif types(i) == 'L'
            tdelta = smax;
        elseif types(i) == 'R'
            tdelta = -smax;
        else
            % do nothing
        end
        % 根据RS曲线路径的输入，基于运动学公式计算RS曲线上每个路径点的状态x, y, th
        for j = 1 : round(abs(segs(i)) / mres) % 四舍五入为最近的小数或整数
           	[px, py, pth] = VehicleDynamic(px, py, pth, s*mres, tdelta, veh.WB); % s*mres中s代表前进和后退
            idx = idx + uint16(1);
            x(idx) = px;
            y(idx) = py;
            th(idx) = pth;
        end
    end
end

function [Open, u16OpenIdx, Close] = Update(wknode, Open, u16OpenIdx, Close, u16CloseIdx, veh, cfg)
    mres = cfg.MOTION_RESOLUTION; % motino resolution    
    smax = veh.MAX_STEER; % 0.6[rad], maximum steering angle
    sres = smax / cfg.N_STEER; % 20, steering resolution  
    % all possible control input，
    for D = [-mres, mres] % D是0.1m, 正负代表前进或后退, 车辆当前位置的后轴中心与下一个位置的后轴中心之间的直线距离，有2个子结点
        for delta = [-smax:sres:-sres, 0, sres:sres:smax] % delta是转向角，分辨率是0.03[rad]，[-0.6, 0.6], 有21个子结点(包含0[rads])
            [isok, tnode] = CalcNextNode(wknode, D, delta, veh, cfg); % 计算wknode的所有子结点，一共2*21=42个，此函数是根据固定的D和delta计算wknode沿着一条路径的所有子结点，tnode是此条路径的末端点
            if isok == false % 子结点不可行
                continue
            end
            [isok, ~] = inNodes(tnode, Close, u16CloseIdx);% 在Close集合中
            if isok
                continue
            end 
            % 拓展的节点如果在Open中比较f值;若不在则添加到Open中
            [isok, idx] = inNodes(tnode, Open, u16OpenIdx);
            if isok
                % 与之前的cost比较，进行更新
                tcost = TotalCost(tnode, cfg);
                ttnode = Open(idx);
                ttcost = TotalCost(ttnode, cfg);
                if tcost < ttcost
                    Open(idx) = tnode;
                end
            else
                u16OpenIdx = u16OpenIdx + uint16(1);
                Open(u16OpenIdx) = tnode;
            end           
        end
    end  
end

function [isok, idx] = inNodes(node, nodes, nodesNum)
    for i = 1 : nodesNum
        tnode = nodes(i);
        if node.xidx == tnode.xidx...
                && node.yidx == tnode.yidx...
                && node.yawidx == tnode.yawidx
            idx = i;
            isok = true;
            return
        end
    end
    idx = nodesNum + uint16(1);
    isok = false;
end

% 根据D和delta计算wknode沿着一条路径的所有子结点
function [isok, tnode] = CalcNextNode(wknode, D, delta, Vehicle, cfg)
    px = wknode.x;
    py = wknode.y;
    pth = wknode.theta;
    
    % 碰撞检测，保证每条轨迹最小长度
    nlist = floor(cfg.MIN_PATH_LENGTH / cfg.MOTION_RESOLUTION) + 1; 
    x = zeros(1, nlist + 1, 'single');
    y = zeros(1, nlist + 1, 'single');
    th = zeros(1, nlist + 1, 'single');
    x(1) = px;
    y(1) = py;
    th(1) = pth;
    for idx = 1 : nlist % 根据当前的状态和给定的控制，计算此条路径上的连着的车辆状态，根据上一时刻计算下一时刻
        [px, py, pth] = VehicleDynamic(px, py, pth, D, delta, Vehicle.WB);
        x(idx + 1) = px; % x, y, th储存了数据，但是没用到变量
        y(idx + 1) = py;
        th(idx + 1) = pth;
        if rem(idx, 5) == 0 % 每隔5个点进行一次碰撞检测
            tvec = [px, py, pth];
            isCollision = VehicleCollisionCheck(tvec, cfg.ObstLine, Vehicle);
            if isCollision
                break
            end
        end
    end
    tnode = wknode;
    if isCollision
        isok = false;
        return
    else
        [isok, xidx, yidx, thidx] = CalcIdx(px, py, pth, cfg); % 把路径末端点的实际坐标转换为栅格坐标
        if isok == false
            return
        else
            cost = wknode.cost;
            if D > 0 % 前进
                cost = cost + cfg.MIN_PATH_LENGTH; 
            else % 后退
                cost = cost + cfg.BACK_COST * cfg.MIN_PATH_LENGTH;
            end
            if D ~= wknode.D
                cost = cost + cfg.SB_COST;
            end
            cost = cost + cfg.STEER_COST * abs(delta);
            cost = cost + cfg.STEER_CHANGE_COST * abs(delta - wknode.delta);
            tnode = Node(xidx, yidx, thidx, D, delta, px, py, pth, ...
                [wknode.xidx, wknode.yidx, wknode.yawidx], cost); % tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本
        end         
    end
end

function [wknode, nodes, idx] = PopNode(nodes, idx, cfg)
    mincost = single(1e9);
    minidx = uint16(1);
    for i = 1 : idx
        tnode = nodes(i);
        % x in the col y in row
        tcost = TotalCost(tnode, cfg);
        if tcost < mincost
            mincost = tcost;
            minidx = i;
        end
    end
    wknode = nodes(minidx);
    for i = minidx + 1 : idx
        nodes(i - 1) = nodes(i);
    end
    idx = idx - uint16(1);
end

function cost = TotalCost(wknode, cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    costmap = cfg.ObstMap;
    cost = cfg.H_COST * costmap(wknode.yidx, wknode.xidx);
    % 当前节点位置到栅格中心的欧式距离
    xshift = wknode.x - (gres * (wknode.xidx - 0.5) + cfg.MINX);
    yshift = wknode.y - (gres * (wknode.yidx - 0.5) + cfg.MINY);
    cost = cost + cfg.H_COST * norm([xshift, yshift]);
    % f = g + h
    cost = wknode.cost + cost;
end

function [isok, path] = AnalysticExpantion(startPose, endPose, veh, cfg)
    isok = true;
    isCollision = false;

    rmin = veh.MIN_CIRCLE;
    smax = veh.MAX_STEER;
    mres = cfg.MOTION_RESOLUTION;
    obstline = cfg.ObstLine;
    
    % 将起点转换到原点计算轨迹，变换坐标系了
    pvec = endPose - startPose;
    phi = mod2pi(startPose(3));
    dcm = [cos(phi), -sin(phi); sin(phi), cos(phi)]; % 起点start坐标系在基坐标系下的方向余弦矩阵
    % dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
    tvec = dcm' * [pvec(1); pvec(2)]; % 计算坐标旋转后各向量在起点start坐标中的表示

    % 看是否从当前点到目标点存在无碰撞的Reeds-Shepp轨迹，前面pvec=End-Start;的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
    path = FindRSPath(tvec(1), tvec(2), pvec(3), veh);

    % 以下是根据路径点和车辆运动学模型计算位置，检测是否会产生碰撞，返回isok的值。对每段路径从起点到终点按顺序进行处理，这一个线段的终点pvec是下一个线段的起点px, py, pth，  
    types = path.type;
    t = rmin * path.t;
    u = rmin * path.u;
    v = rmin * path.v;
    w = rmin * path.w;
    x = rmin * path.x;
    segs = [t, u, v, w, x];
    pvec = startPose;
    for i = 1 : 5
        if segs(i) ==0
            continue
        end
        px =pvec(1);
        py = pvec(2);
        pth = pvec(3);
        s = sign(segs(i)); % 符号函数, 判断此段运动方向是前进还是后退
        
        % 根据车辆的2*3种运动类型(前后2种，转向3种)，设置D和delta
        D = s * mres; % 分辨率的正负
        if types(i) == 'S'
            delta = 0;
        elseif types(i) == 'L'
            delta = smax;
        elseif types(i) == 'R'
            delta = -smax;
        else
            % do nothing
        end
        
        % 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
        for idx = 1 : round(abs(segs(i)) / mres) % round()四舍五入
            % D和delta是固定，说明转弯的时候是按固定半径的圆转弯
           	[px, py, pth] = VehicleDynamic(px, py, pth, D, delta, veh.WB);
            if rem(idx, 5) == 0 % rem(a, b)，返回用 a / b后的余数，每5个点，即0.5m检查下是否碰撞
                tvec = [px, py, pth];
                isCollision = VehicleCollisionCheck(tvec, obstline, veh);
                if isCollision
                    break
                end
            end
         end
        if isCollision
            isok = false;
            break % 如果路径存在碰撞则舍弃此条Reeds-Shepp路径
        end
        pvec = [px, py, pth];
    end
    % 终点位姿小于期望阈值也舍弃
    if (mod2pi(pth) - endPose(3)) > deg2rad(5)
        isok = false;
    end
end

% 根据当前位姿和输入, 计算下一位置的位姿
% x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t=D
function [x, y, theta] = VehicleDynamic(x, y, theta, D, delta, L)
    x = x + D * cos(theta);
    y = y + D * sin(theta);
    theta = theta + D / L * tan(delta); % L是轴距, 航向变化, theta_dot=v / R, R=L / tan(delta)
    theta = mod2pi(theta);
end

% 把位姿(x, y, theta)转换为grid上的栅格索引, 如果不符合实际则isok=false
function [isok, xidx, yidx, thidx] = CalcIdx(x, y, theta, cfg)
    gres = cfg.XY_GRID_RESOLUTION;
    yawres = cfg.YAW_GRID_RESOLUTION;
    xidx = ceil((x - cfg.MINX) / gres);
    yidx = ceil((y - cfg.MINY) / gres);
    theta = mod2pi(theta);
    thidx = ceil((theta - cfg.MINYAW) / yawres);
    isok = true;
    if xidx <=0 || xidx > ceil((cfg.MAXX-cfg.MINX) / gres)
        isok = false;
        return
    elseif yidx <=0 || yidx > ceil((cfg.MAXY-cfg.MINY) / gres)
        isok = false;
        return
    end
    costmap = cfg.ObstMap;
    if costmap(yidx, xidx) == inf
        isok = false;
    end
end

function v = mod2pi(x)
    v = rem(x, 2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end