clc; clear; close all;

segment1 = [0, 0, 5, 3];
segment2 = [6, 4, 7, 9];
plot([segment1(1), segment1(3)], [segment1(2), segment1(4)]);
hold on;
plot([segment2(1), segment2(3)], [segment2(2), segment2(4)])
[bCross, x, y] = HasCross(segment1, segment2)
scatter(x, y, 25, "red")

function [bCross, x, y] = HasCross(segment1, segment2)
%HASCROSS 判断两线段是否相交
% segmeng1 - 线段1起点和终点坐标
% segment2 - 线段2起点和终点坐标

    x_a = segment1(1);
    x_b = segment1(3);
    x_c = segment2(1);
    x_d = segment2(3);
    y_a = segment1(2);
    y_b = segment1(4);
    y_c = segment2(2);
    y_d = segment2(4);

    x = single(0);
    y = single(0);
    % step1 计算行列式
    delta = (x_d - x_c) * (y_b - y_a) - (x_b - x_a) * (y_d - y_c);
    if abs(delta) < single(1e-6)
        bCross = false;
        return;
    end
    % step2 判断是否有交点
    lambda = ((x_d - x_c) * (y_c - y_a) - (x_c - x_a) * (y_d - y_c)) / delta;
    if lambda < single(0) || lambda > single(1)
        bCross = false;
        return;
    end
    miu = ((x_b - x_a) * (y_c - y_a) - (x_c - x_a) * (y_b - y_a)) / delta;
    if miu < single(0) || miu > single(1)
        bCross = false;
        return;
    end
    bCross = true;
    % step3 计算交点
    x = x_a + lambda * (x_b - x_a);
    y = y_a + lambda * (y_b - y_a);
end