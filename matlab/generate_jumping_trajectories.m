clc
clear all
close all

duty = 0.3;

num_points = 1000;
y_nom = -0.134; % nominal y position of standing
short_max = -0.16; % y position of small jumps
big_max = -0.2; % y position of big jumps

start_jump = num_points*duty/2;
t_crouch = 1:num_points*0.5*(1-duty)-1;
t_jump = t_crouch(end)+1:t_crouch(end)+num_points*duty;
t_recover = t_jump(end)+1:num_points;

t_vec = [t_crouch t_jump t_recover ];

x_small_straight = zeros(1,num_points);
y_small_straight = [linspace(y_nom,-0.1,numel(t_crouch)) ...
    linspace(-0.1,short_max,numel(t_jump))...
    linspace(short_max,y_nom,numel(t_recover))];

jump_small_straight_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_straight.' ...
    zeros(length(t_vec),1) y_small_straight.', zeros(length(t_vec),6)];

jump_small_straight_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_straight.' ...
    zeros(length(t_vec),1) y_small_straight.', zeros(length(t_vec),6)];

jump_small_straight_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_straight.' ...
    zeros(length(t_vec),1) y_small_straight.', zeros(length(t_vec),6)];

jump_small_straight_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_straight.' ...
    zeros(length(t_vec),1) y_small_straight.', zeros(length(t_vec),6)];

jump_small_straight = [jump_small_straight_0;
    jump_small_straight_1;
    jump_small_straight_2;
    jump_small_straight_3];

writematrix(jump_small_straight,"jump_small_straight.txt","Delimiter"," ")


x_big_straight = zeros(1,num_points);
y_big_straight = [linspace(y_nom,-0.1,numel(t_crouch)) ...
    linspace(-0.1,big_max,numel(t_jump))...
    linspace(big_max,y_nom,numel(t_recover))];

jump_big_straight_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_straight.' ...
    zeros(length(t_vec),1) y_big_straight.', zeros(length(t_vec),6)];

jump_big_straight_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_straight.' ...
    zeros(length(t_vec),1) y_big_straight.', zeros(length(t_vec),6)];

jump_big_straight_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_straight.' ...
    zeros(length(t_vec),1) y_big_straight.', zeros(length(t_vec),6)];

jump_big_straight_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_straight.' ...
    zeros(length(t_vec),1) y_big_straight.', zeros(length(t_vec),6)];

jump_big_straight = [jump_big_straight_0;
    jump_big_straight_1;
    jump_big_straight_2;
    jump_big_straight_3];

writematrix(jump_big_straight,"jump_big_straight.txt","Delimiter"," ")

theta = deg2rad(20);

x_small_angle_20 = [linspace(0.0,0.0,numel(t_crouch)) ...
    linspace(0.0,short_max*sin(theta),numel(t_jump))...
    linspace(short_max*sin(theta),0.0,numel(t_recover))];
y_small_angle_20 = [linspace(y_nom,-0.1,numel(t_crouch)) ...
    linspace(-0.1,short_max*cos(theta),numel(t_jump))...
    linspace(short_max*cos(theta),y_nom,numel(t_recover))];

jump_small_angle_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_20 = [jump_small_angle_0;
    jump_small_angle_1;
    jump_small_angle_2;
    jump_small_angle_3];

writematrix(jump_small_angle_20,"jump_small_angle_20.txt","Delimiter"," ")


x_big_angle_20 = [linspace(0.0,0.0,numel(t_crouch)) ...
    linspace(0.0,short_max*sin(theta),numel(t_jump))...
    linspace(short_max*sin(theta),0.0,numel(t_recover))];
y_big_angle_20 = [linspace(y_nom,-0.1,numel(t_crouch)) ...
    linspace(-0.1,big_max*cos(theta),numel(t_jump))...
    linspace(big_max*cos(theta),y_nom,numel(t_recover))];

jump_big_angle_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_20 = [jump_big_angle_0;
    jump_big_angle_1;
    jump_big_angle_2;
    jump_big_angle_3];

writematrix(jump_big_angle_20,"jump_big_angle_20.txt","Delimiter"," ")


x_small_angle_20 = [linspace(0.0,0.0,numel(t_crouch)) ...
    linspace(0.0,short_max*sin(theta),numel(t_jump))...
    linspace(short_max*sin(theta),0.0,numel(t_recover))];
y_small_angle_20 = [linspace(-0.1,-0.1,numel(t_crouch)) ...
    linspace(-0.1,short_max*cos(theta),numel(t_jump))...
    linspace(short_max*cos(theta),-0.1,numel(t_recover))];

jump_small_angle_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_small_angle_20.' ...
    zeros(length(t_vec),1) y_small_angle_20.', zeros(length(t_vec),6)];

jump_small_angle_20 = [jump_small_angle_0;
    jump_small_angle_1;
    jump_small_angle_2;
    jump_small_angle_3];

writematrix(jump_small_angle_20,"jump_small_angle_20_2.txt","Delimiter"," ")


x_big_angle_20 = [linspace(0.0,0.0,numel(t_crouch)) ...
    linspace(0.0,short_max*sin(theta),numel(t_jump))...
    linspace(short_max*sin(theta),0.0,numel(t_recover))];
y_big_angle_20 = [linspace(-0.1,-0.1,numel(t_crouch)) ...
    linspace(-0.1,big_max*cos(theta),numel(t_jump))...
    linspace(big_max*cos(theta),-0.1,numel(t_recover))];

jump_big_angle_0 = [t_vec.' zeros(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_1 = [t_vec.' ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_2 = [t_vec.' 2*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_3 = [t_vec.' 3*ones(length(t_vec),1) ...
    3*ones(length(t_vec),1) ones(length(t_vec),1) x_big_angle_20.' ...
    zeros(length(t_vec),1) y_big_angle_20.', zeros(length(t_vec),6)];

jump_big_angle_20 = [jump_big_angle_0;
    jump_big_angle_1;
    jump_big_angle_2;
    jump_big_angle_3];

writematrix(jump_big_angle_20,"jump_big_angle_20_2.txt","Delimiter"," ")











% jump_small_straight = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 0 3 1 0.00 0 -0.16 0 0 0 0 0 0;
%     500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 1 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 1 3 1 0.00 0 -0.16 0 0 0 0 0 0;
%     500 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 2 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 2 3 1 0.00 0 -0.16 0 0 0 0 0 0;
%     500 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 3 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 3 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 3 3 1 0.00 0 -0.16 0 0 0 0 0 0;
%     500 3 3 1 0.00 0 -0.134 0 0 0 0 0 0];
% 
% writematrix(jump_small_straight,"jump_small_straight.txt","Delimiter"," ")
% 
% jump_big_straight = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 0 3 1 0.00 0 -0.2 0 0 0 0 0 0;
%     500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 1 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 1 3 1 0.00 0 -0.2 0 0 0 0 0 0;
%     500 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 2 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 2 3 1 0.00 0 -0.2 0 0 0 0 0 0;
%     500 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 3 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 3 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 3 3 1 0.00 0 -0.2 0 0 0 0 0 0;
%     500 3 3 1 0.00 0 -0.134 0 0 0 0 0 0];
% 
% writematrix(jump_big_straight,"jump_big_straight.txt","Delimiter"," ")
% 
% theta = deg2rad(20);
% 
% jump_small_angle_20 = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 0 3 1 -0.16*sin(theta) 0 -0.16*cos(theta) 0 0 0 0 0 0;
%     500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 1 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 1 3 1 -0.16*sin(theta) 0 -0.16*cos(theta) 0 0 0 0 0 0;
%     500 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 2 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 2 3 1 -0.16*sin(theta) 0 -0.16*cos(theta) 0 0 0 0 0 0;
%     500 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 3 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 3 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 3 3 1 -0.16*sin(theta) 0 -0.16*cos(theta) 0 0 0 0 0 0;
%     500 3 3 1 0.00 0 -0.134 0 0 0 0 0 0];
% 
% writematrix(jump_small_angle_20,"jump_small_angle_20.txt","Delimiter"," ")
% 
% jump_big_angle_20 = [ 0 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 0 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 0 3 1 -0.2*sin(theta) 0 -0.2*cos(theta) 0 0 0 0 0 0;
%     500 0 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 1 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 1 3 1 -0.2*sin(theta) 0 -0.2*cos(theta) 0 0 0 0 0 0;
%     500 1 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 2 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 2 3 1 -0.2*sin(theta) 0 -0.2*cos(theta) 0 0 0 0 0 0;
%     500 2 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     0 3 3 1 0.00 0 -0.134 0 0 0 0 0 0;
%     250*(1-duty) 3 3 1 0.00 0 -0.1 0 0 0 0 0 0;
%     300 3 3 1 -0.2*sin(theta) 0 -0.2*cos(theta) 0 0 0 0 0 0;
%     500 3 3 1 0.00 0 -0.134 0 0 0 0 0 0];
% 
% writematrix(jump_big_angle_20,"jump_big_angle_20.txt","Delimiter"," ")
