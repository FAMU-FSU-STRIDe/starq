% Load in Tracker graph Data to find vel averages etc etc
% Ash Chase
% 2-6-24

close all
clear all
clc

%% Robot parameters

m = 7.85; %kg - only needed for drag calcs/forces/etc 

%% Load in 

filename1 = 'test1_1p2.txt';
Data1 = readmatrix(filename1);

t1 = Data1(:,1); %time in s
x1 = Data1(:,2); %x direction in ft
y1 = Data1(:,3); %y direction in ft
r1 = Data1(:,4); %relative postion in ft
vx1 = Data1(:,5); %x velocity in ft/s
vy1 = Data1(:,6); %y velocity in ft/s
v1 = Data1(:,7); %relative velocity in ft/s
ax1 = Data1(:,8);
ay1 = Data1(:,9);
a1 = Data1(:,10);

avg1 = mean(v1(2:end-1))

filename2 = 'test2_1p2.txt';
Data2 = readmatrix(filename2);

t2 = Data2(:,1); %time in s
x2 = Data2(:,2); %x direction in ft
y2 = Data2(:,3); %y direction in ft
r2 = Data2(:,4); %relative postion in ft
vx2 = Data2(:,5); %x velocity in ft/s
vy2 = Data2(:,6); %y velocity in ft/s
v2 = Data2(:,7); %relative velocity in ft/s
ax2 = Data2(:,8);
ay2 = Data2(:,9);
a2 = Data2(:,10);

avg2 = mean(v2(2:end-1))

filename3 = 'test3_1p2.txt';
Data3 = readmatrix(filename3);

t3 = Data3(:,1); %time in s
x3 = Data3(:,2); %x direction in ft
y3 = Data3(:,3); %y direction in ft
r3 = Data3(:,4); %relative postion in ft
vx3 = Data3(:,5); %x velocity in ft/s
vy3 = Data3(:,6); %y velocity in ft/s
v3 = Data3(:,7); %relative velocity in ft/s
ax3 = Data3(:,8);
ay3 = Data3(:,9);
a3 = Data3(:,10);

avg3 = mean(v3(2:end-1))


avgVel_1p2 = mean([avg1,avg2,avg3])

% convert to metric - if messed up in tracker
% x = 0.3048.*x;
% y = 0.3048.*y;
% r = 0.3048.*r;
% vx = 0.3048.*vx;
% vy = 0.3048.*vy;
% v = 0.3048.*v;
% ax = 0.3048.*ax;
% ay = 0.3048.*ay;
% a = 0.3048.*a;


% figure()
% subplot(2,1,1)
% plot(t(3:end,:), rF(3:end,:))
% xlabel('time(s)')
% ylabel('rel postion (foot)')
% 
% subplot(2,1,2)
% plot(t(3:end,:), vF(3:end,:), 'k-')
% hold on
% plot(t(3:end,:), vK(3:end,:), 'r-')
% xlabel('time(s)')
% ylabel('rel vel')
% legend('foot', 'knee')

% subplot(3,1,3)
% plot(t(3:end,:), vK(3:end,:))
% xlabel('time(s)')
% ylabel('rel vel (knee)')

