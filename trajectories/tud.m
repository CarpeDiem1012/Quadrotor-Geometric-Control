% This document shows the final trajectory of the quadrotor, TUD. 
% It includes the coordinates of each trajectory and the diagram of 
% the trajectory 
% 
% Written by Jingyue Liu
% Email: J.Liu-28@student.tudelft.nl, 
% Second year master student of robotics, 
% Teaching assistant of RO47001-21/22

clear all;
close all;
% The coordinate position of the letter T
% First from (0, 0) to (0, 7)
y1 = 0:0.1:7;
x1 = 0*y1;
% from (0, 7) to (-2, 7)
x2 = 0:-0.1:-2;
y2 = 0*x2+7;
% from (-2, 7) to (2, 7)
x3 = -2:0.1:2;
y3 = 0*x3+7;

% The coordinate position of the letter U
% In order to simplify the trajectory, the letter U consists of two vertical 
% lines with a length of 5 and a semicircle whose radius is 2
% line 1
y5 = 7:-0.1:2;
x5 = y5*0 + 3;
% semicircle. The function is (x-5)^2 + (y-2)^2 = 2^2
x6 = 3:0.1:7; 
y6 = -sqrt(4-(x6-5).^2)+2;
% line 2
y7 = 2:0.1:7;
x7 = 0*y7+7;

% The coordinate position of the letter D
% Similarly, the letter D consists of four trajectories, which are a 
% vertical line with a length of 7, two horizontal lines with a length of 
% 0.5, and a semicircle with a radius of 3.5.

% The long verticle line 
y9 = 7:-0.1:0;
x9 = 0*y9 + 8;
% Upper horizontal line
x11 = 8:0.1:8.5;
y11 = 0*x11 + 7;
% semicircle. The function is (x-8.5)^2 + (y-3.5)^2 = 3.5 ^2 
y10 = 7:-0.1:0;
x10 = sqrt(3.5*3.5 - (y10-3.5).^2) +8.5;
% Horizontal line below
x12 = 8.5:-0.1:8;
y12 = 0*x12;

axis equal
hold on
plot(x1,y1, 'cyan-', 'LineWidth',2);
plot(x2,y2, 'cyan-', 'LineWidth',2);
plot(x3,y3, 'cyan-', 'LineWidth',2);
plot(x5,y5, 'cyan-', 'LineWidth',2);
plot(x6,y6, 'cyan-', 'LineWidth',2);
plot(x7,y7, 'cyan-', 'LineWidth',2);
plot(x9,y9, 'cyan-', 'LineWidth',2);
plot(x11,y11,'cyan-', 'LineWidth',2)
plot(x10,y10, 'cyan-', 'LineWidth',2);
plot(x12,y12,'cyan-', 'LineWidth',2);
axis([-5 15 -1 9]);
hold off
