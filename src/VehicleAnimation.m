function VehicleAnimation(x, y, theta, cfg, veh, keep)
    sz=get(0,'screensize');
    figure('outerposition', sz);
    videoFWriter = VideoWriter('Parking.mp4', 'MPEG-4');
    open(videoFWriter);
    ObstList = cfg.ObstList;
    scatter(ObstList(:, 1), ObstList(:, 2), 10, 'r') % ��ɢ��ͼ
    hold on; grid on; grid minor;
    axis equal
    set(gca,'YDir','reverse');
    xlim([cfg.MINX, cfg.MAXX]);
    ylim([cfg.MINY, cfg.MAXY]);
    plot(x, y, 'b') % �滮�����Ĺ켣����ɫ����   
    px = x(1);
    py = y(1);
    pth = theta(1);
    [vehx, vehy] = getVehTran([px, py, pth], veh); % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
    last_vehx = vehx;
    last_vehy = vehy;
    h1 = plot(vehx, vehy, 'r'); % �����߿�
    h2 = plot(px, px, 'gx', 'MarkerSize', 10); % ������������
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2 : length(theta)
        px = x(i);
        py = y(i);
        pth = theta(i);
        [vehx, vehy] = getVehTran([px, py, pth], veh);
        if keep
            plot(last_vehx, last_vehy, 'Color', [0.5 0.5 0.5]); % �����߿�
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

 % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
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
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [CornerFLInLocal(1),CornerFRInLocal(1),CornerRRInLocal(1),CornerRLInLocal(1),CornerFLInLocal(1)];
    y = [CornerFLInLocal(2),CornerFRInLocal(2),CornerRRInLocal(2),CornerRLInLocal(2),CornerFLInLocal(2)];
end