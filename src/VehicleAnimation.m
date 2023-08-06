function VehicleAnimation(x, y, theta, cfg, veh, keep)
    sz=get(0,'screensize');
    figure('outerposition', sz);
    videoFWriter = VideoWriter('Parking.mp4', 'MPEG-4');
    open(videoFWriter);
    ObstList = cfg.ObstList;
    scatter(ObstList(:, 1), ObstList(:, 2), 10, 'r') % 画散点图
    hold on; grid on; grid minor;
    axis equal
    set(gca,'YDir','reverse');
    xlim([cfg.MINX, cfg.MAXX]);
    ylim([cfg.MINY, cfg.MAXY]);
    plot(x, y, 'b') % 规划出来的轨迹，蓝色曲线   
    px = x(1);
    py = y(1);
    pth = theta(1);
    [vehx, vehy] = getVehTran([px, py, pth], veh); % 根据后轴中心的位姿计算车辆边框的位姿
    last_vehx = vehx;
    last_vehy = vehy;
    h1 = plot(vehx, vehy, 'r'); % 车辆边框
    h2 = plot(px, px, 'gx', 'MarkerSize', 10); % 车辆后轴中心
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2 : length(theta)
        px = x(i);
        py = y(i);
        pth = theta(i);
        [vehx, vehy] = getVehTran([px, py, pth], veh);
        if keep
            plot(last_vehx, last_vehy, 'Color', [0.5 0.5 0.5]); % 车辆边框
            last_vehx = vehx;
            last_vehy = vehy;
        end
        h1.XData = vehx;
        h1.YData = vehy;
        h2.XData = px; 
        h2.YData = py;
        img = getframe(gcf);
        writeVideo(videoFWriter,img);
        if i == length(theta)
            pause(1)
        end
    end
    close(videoFWriter);
end

 % 根据后轴中心的位姿计算车辆边框的位姿
function [x, y] = getVehTran(vec, veh)

    CornerFLInChasis = [veh.LF; 0.5 * veh.W];
    CornerFRInChasis = [veh.LF; -0.5 * veh.W];
    CornerRLInChasis = [-veh.LB; 0.5 * veh.W];
    CornerRRInChasis = [-veh.LB; -0.5 * veh.W];
    R = [cos(vec(3)), -sin(vec(3)); sin(vec(3)), cos(vec(3))];
    CornerFLInLocal = [vec(1); vec(2)] + R * CornerFLInChasis;
    CornerFRInLocal = [vec(1); vec(2)] + R * CornerFRInChasis;
    CornerRLInLocal = [vec(1); vec(2)] + R * CornerRLInChasis;
    CornerRRInLocal = [vec(1); vec(2)] + R * CornerRRInChasis;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [CornerFLInLocal(1),CornerFRInLocal(1),CornerRRInLocal(1),CornerRLInLocal(1),CornerFLInLocal(1)];
    y = [CornerFLInLocal(2),CornerFRInLocal(2),CornerRRInLocal(2),CornerRLInLocal(2),CornerFLInLocal(2)];
end