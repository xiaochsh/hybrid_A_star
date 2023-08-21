function [x, y, th] = HybridAStar(startPose, endPose, veh, cfg)
    mres = cfg.MOTION_RESOLUTION; % motino resolution 
    
    % ����ʼ��λ��(x, y, theta)ת��Ϊgrid�ϵ�դ������
    [isok, xidx, yidx, thidx] = CalcIdx(startPose(1), startPose(2), startPose(3), cfg);
    if isok % ��λ��դ����Ϊһ����㣬�γ������ṹ
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
        
        % �ж��Ƿ���Close�����ڣ�������wknode��ֲ����ĩβ
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
            % ��wknodeΪ������ɵ��յ������ײRS����
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
            break % �����ֱ����������ײRS����RS���ߣ�������whileѭ��
        end
        [Open, u16OpenIdx, Close] = Update(wknode, Open, u16OpenIdx, Close, u16CloseIdx, veh, cfg); % ʹ��
        iter = iter + uint8(1);
    end
end

function [x, y, th] = getFinalPath(path, Close, u16CloseIdx, veh, cfg)
    wknode = Close(u16CloseIdx); % Close�������ڵ�Ϊ����ײRS�������
    u16CloseIdx = u16CloseIdx - uint16(1);
    nodes = repmat(Node(0, 0, 0, 0, 0, 0, 0, 0, [0, 0, 0], 0), 1000, 1);
    nodesNum = uint16(1);
    nodes(nodesNum) = wknode;
    % ��Ŀ���wknode��parent, ���ݣ�ֱ��Close����Ϊ��
    while u16CloseIdx > uint16(0)
        parent = wknode.parent;
        % �����Ŀ�귵�ص���ʼ���·�������У�����nodes��
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
    
    % ���յ�·����1-ֻ��A*·����2-ֻ��RS����·����3-���A*·��
    % ·��Ҫô�Ǵ�RS·����Ҫô����RS·���ͻ��A*���һ������·�����ȴ������A*�Ľ�㣬�����RS·�����϶���RS·��
    if nodesNum > uint16(1)
        % nodes�е�һ������ΪRS���ߵ���㣬�����λ���A*·��
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
        % ���һ������Ǵ���RS·���յ�
        tnode = nodes(1);
        idx = idx + uint16(1);
        x(idx) = tnode.x;
        y(idx) = tnode.y;
        th(idx) = tnode.theta;
    end
    
    % RS���ߵ���㲻����켣
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
        s = sign(segs(i)); % ǰ�������
        if types(i) == 'S'          
            tdelta = 0;
        elseif types(i) == 'L'
            tdelta = smax;
        elseif types(i) == 'R'
            tdelta = -smax;
        else
            % do nothing
        end
        % ����RS����·�������룬�����˶�ѧ��ʽ����RS������ÿ��·�����״̬x, y, th
        for j = 1 : round(abs(segs(i)) / mres) % ��������Ϊ�����С��������
           	[px, py, pth] = VehicleDynamic(px, py, pth, s*mres, tdelta, veh.WB); % s*mres��s����ǰ���ͺ���
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
    % all possible control input��
    for D = [-mres, mres] % D��0.1m, ��������ǰ�������, ������ǰλ�õĺ�����������һ��λ�õĺ�������֮���ֱ�߾��룬��2���ӽ��
        for delta = [-smax:sres:-sres, 0, sres:sres:smax] % delta��ת��ǣ��ֱ�����0.03[rad]��[-0.6, 0.6], ��21���ӽ��(����0[rads])
            [isok, tnode] = CalcNextNode(wknode, D, delta, veh, cfg); % ����wknode�������ӽ�㣬һ��2*21=42�����˺����Ǹ��ݹ̶���D��delta����wknode����һ��·���������ӽ�㣬tnode�Ǵ���·����ĩ�˵�
            if isok == false % �ӽ�㲻����
                continue
            end
            [isok, ~] = inNodes(tnode, Close, u16CloseIdx);% ��Close������
            if isok
                continue
            end 
            % ��չ�Ľڵ������Open�бȽ�fֵ;�����������ӵ�Open��
            [isok, idx] = inNodes(tnode, Open, u16OpenIdx);
            if isok
                % ��֮ǰ��cost�Ƚϣ����и���
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

% ����D��delta����wknode����һ��·���������ӽ��
function [isok, tnode] = CalcNextNode(wknode, D, delta, Vehicle, cfg)
    px = wknode.x;
    py = wknode.y;
    pth = wknode.theta;
    
    % ��ײ��⣬��֤ÿ���켣��С����
    nlist = floor(cfg.MIN_PATH_LENGTH / cfg.MOTION_RESOLUTION) + 1; 
    x = zeros(1, nlist + 1, 'single');
    y = zeros(1, nlist + 1, 'single');
    th = zeros(1, nlist + 1, 'single');
    x(1) = px;
    y(1) = py;
    th(1) = pth;
    for idx = 1 : nlist % ���ݵ�ǰ��״̬�͸����Ŀ��ƣ��������·���ϵ����ŵĳ���״̬��������һʱ�̼�����һʱ��
        [px, py, pth] = VehicleDynamic(px, py, pth, D, delta, Vehicle.WB);
        x(idx + 1) = px; % x, y, th���������ݣ�����û�õ�����
        y(idx + 1) = py;
        th(idx + 1) = pth;
        if rem(idx, 5) == 0 % ÿ��5�������һ����ײ���
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
        [isok, xidx, yidx, thidx] = CalcIdx(px, py, pth, cfg); % ��·��ĩ�˵��ʵ������ת��Ϊդ������
        if isok == false
            return
        else
            cost = wknode.cost;
            if D > 0 % ǰ��
                cost = cost + cfg.MIN_PATH_LENGTH; 
            else % ����
                cost = cost + cfg.BACK_COST * cfg.MIN_PATH_LENGTH;
            end
            if D ~= wknode.D
                cost = cost + cfg.SB_COST;
            end
            cost = cost + cfg.STEER_COST * abs(delta);
            cost = cost + cfg.STEER_CHANGE_COST * abs(delta - wknode.delta);
            tnode = Node(xidx, yidx, thidx, D, delta, px, py, pth, ...
                [wknode.xidx, wknode.yidx, wknode.yawidx], cost); % tnode��·����ĩ�˵㣬costΪ����ǰ״̬����·��ĩ��״̬�ĳɱ�
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
    % ��ǰ�ڵ�λ�õ�դ�����ĵ�ŷʽ����
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
    
    % �����ת����ԭ�����켣���任����ϵ��
    pvec = endPose - startPose;
    phi = mod2pi(startPose(3));
    dcm = [cos(phi), -sin(phi); sin(phi), cos(phi)]; % ���start����ϵ�ڻ�����ϵ�µķ������Ҿ���
    % dcm*x ��ʾ���������е�x��ʾ����ת�������ϵ�У�������������ת����������������еı�ʾ
    tvec = dcm' * [pvec(1); pvec(2)]; % ����������ת������������start�����еı�ʾ

    % ���Ƿ�ӵ�ǰ�㵽Ŀ����������ײ��Reeds-Shepp�켣��ǰ��pvec=End-Start;������������ע�⣡�����x, y, prev(3)�ǰ����ת������startΪԭ������ϵ�µ�����
    path = FindRSPath(tvec(1), tvec(2), pvec(3), veh);

    % �����Ǹ���·����ͳ����˶�ѧģ�ͼ���λ�ã�����Ƿ�������ײ������isok��ֵ����ÿ��·������㵽�յ㰴˳����д�������һ���߶ε��յ�pvec����һ���߶ε����px, py, pth��  
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
        s = sign(segs(i)); % ���ź���, �жϴ˶��˶�������ǰ�����Ǻ���
        
        % ���ݳ�����2*3���˶�����(ǰ��2�֣�ת��3��)������D��delta
        D = s * mres; % �ֱ��ʵ�����
        if types(i) == 'S'
            delta = 0;
        elseif types(i) == 'L'
            delta = smax;
        elseif types(i) == 'R'
            delta = -smax;
        else
            % do nothing
        end
        
        % �Ѵ˶ε�·����ɢ��Ϊ·�㣬��դ������, Ȼ��Ϊ·�㣬Ȼ�����Ƿ�����ϰ�����ײ����
        for idx = 1 : round(abs(segs(i)) / mres) % round()��������
            % D��delta�ǹ̶���˵��ת���ʱ���ǰ��̶��뾶��Բת��
           	[px, py, pth] = VehicleDynamic(px, py, pth, D, delta, veh.WB);
            if rem(idx, 5) == 0 % rem(a, b)�������� a / b���������ÿ5���㣬��0.5m������Ƿ���ײ
                tvec = [px, py, pth];
                isCollision = VehicleCollisionCheck(tvec, obstline, veh);
                if isCollision
                    break
                end
            end
         end
        if isCollision
            isok = false;
            break % ���·��������ײ����������Reeds-Shepp·��
        end
        pvec = [px, py, pth];
    end
    % �յ�λ��С��������ֵҲ����
    if (mod2pi(pth) - endPose(3)) > deg2rad(5)
        isok = false;
    end
end

% ���ݵ�ǰλ�˺�����, ������һλ�õ�λ��
% x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), �ڲ���ʱ��t��, ����x = x + v_x * t * cos(theta)������v_x * t=D
function [x, y, theta] = VehicleDynamic(x, y, theta, D, delta, L)
    x = x + D * cos(theta);
    y = y + D * sin(theta);
    theta = theta + D / L * tan(delta); % L�����, ����仯, theta_dot=v / R, R=L / tan(delta)
    theta = mod2pi(theta);
end

% ��λ��(x, y, theta)ת��Ϊgrid�ϵ�դ������, ���������ʵ����isok=false
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
    if costmap(yidx, xidx) > single(1e9) - single(0.001)
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