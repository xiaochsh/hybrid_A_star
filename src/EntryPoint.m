clear; close all

Vehicle.WB = 2.785;  % [m] wheel base: rear to front steer
Vehicle.W = 1.887; % [m] width of vehicle
Vehicle.LF = 3.7; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.029; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB / tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius

% Motion resolution define
Configure.MOTION_RESOLUTION = 0.05; % [m] path interporate resolution
Configure.N_STEER = 20.0; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 1.0; % [m]
Configure.YAW_GRID_RESOLUTION = deg2rad(45); % [rad]
% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.BACK_COST = 1.5; % backward penalty cost
Configure.STEER_CHANGE_COST = 1.5; % steer angle change penalty cost
Configure.STEER_COST = 1.5; % steer angle change penalty cost
Configure.H_COST = 10; % Heuristic cost
% Path related define
Configure.MIN_PATH_LENGTH = 1.5 * Configure.XY_GRID_RESOLUTION;

% Park lot line for collision check
%start_x start_y end_x end_y
ObstLine = [-10, 11, 10, 11; 
            -10, 5, -1.5, 5;
            -1.5, 0, 1.5, 0;
            1.5, 5, 10, 5;
            -10, 5, -10, 11;
            -1.5, 0, -1.5, 5;
            1.5, 0, 1.5, 5;
            10, 5, 10, 11];

ObstList = []; % Obstacle point list
for i = 1 : size(ObstLine, 1)
    if abs(ObstLine(i, 3) - ObstLine(i, 1)) < single(1e-6)
        theta = 0.5 * pi;
    else
        theta = atan2(ObstLine(i, 4) - ObstLine(i, 2), ObstLine(i, 3) - ObstLine(i, 1));
    end
    dist = single(0);
    while true
        if dist > norm(ObstLine(i, [3 4]) - ObstLine(i, [1 2]))
            ObstList(end + 1, :) = [ObstLine(i, 3), ObstLine(i, 4)];
            break;
        else
            ObstList(end + 1, :) = [ObstLine(i, 1) +  dist * cos(theta), ObstLine(i, 2) + dist * sin(theta)];
            dist = dist + Configure.XY_GRID_RESOLUTION;
        end
    end
end

if false
    figure(); hold on; grid on; axis equal;
    for i = 1 : size(ObstLine, 1)
        plot(ObstLine(i, [1 3]), ObstLine(i, [2 4]))
    end
    for i = 1 : size(ObstList, 1)
        scatter(ObstList(i, 1), ObstList(i, 2))
    end
end

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;
% Grid bound
Configure.MINX = min(ObstList(:, 1)) - Configure.EXTEND_AREA;
Configure.MAXX = max(ObstList(:, 1)) + Configure.EXTEND_AREA;
Configure.MINY = min(ObstList(:, 2)) - Configure.EXTEND_AREA;
Configure.MAXY = max(ObstList(:, 2)) + Configure.EXTEND_AREA;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;

startPose = [-3, 7, pi / 30];
goalPose = [0, 1.2, pi / 2];

% 使用完整约束有障碍情况下用A*搜索的最短路径最为hybrid A*的启发值
Configure.ObstMap = GridAStar(Configure.ObstList, goalPose, Configure.XY_GRID_RESOLUTION);
[x, y, th] = HybridAStar(startPose, goalPose, Vehicle, Configure);
% GridAStar(ObstList,End,2);
if isempty(x)
    disp("Failed to find path!")
else
    VehicleAnimation(x, y, th, Configure, Vehicle, true)
end