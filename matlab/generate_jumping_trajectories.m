clc
clear all
close all

jump_small_straight = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
    200 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
    300 0 3 1 0.00 0 -0.16 0 0 0 0 0 0;
    500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0];

writematrix(jump_small_straight,"jump_small_straight.txt")

jump_big_straight = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
    200 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
    300 0 3 1 0.00 0 -0.2 0 0 0 0 0 0;
    500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0];

writematrix(jump_big_straight,"jump_big_straight.txt")

theta = deg2rad(20);

jump_small_angle_20 = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
    200 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
    300 0 3 1 -0.16*sin(theta) 0 -0.16*cos(theta) 0 0 0 0 0 0;
    500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0];

writematrix(jump_small_angle_20,"jump_small_angle_20.txt")

jump_big_angle_20 = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
    200 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
    300 0 3 1 -0.2*sin(theta) 0 -0.2*cos(theta) 0 0 0 0 0 0;
    500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0];

writematrix(jump_big_angle_20,"jump_big_angle_20.txt")
