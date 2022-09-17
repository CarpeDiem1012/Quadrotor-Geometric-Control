function [desired_state] = tudtrj(t)
% CIRCLE trajectory generator for a circle

% % =================== Your code goes here ===================
% % You have to set the pos, vel, acc, yaw and yawdot variables
% % NOTE: the simulator will spawn the robot to be at the
% %       position you return for t == 0
% 
% pos = [0; 0; 0];
% vel = [0; 0; 0];
% acc = [0; 0; 0];
% yaw = 0;
% yawdot = 0;
% 
% 
% % =================== Your code ends here ===================

%% A sampled solution
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
T = 17; % total time you expect to finish the task
dt = 0.0001;
T_t = 5;
% T_t2u = 0.5;
T_u = 6.5;
% T_u2d = 0.5;
T_d = 5.5;
R_u = 2;
R_d = 3.5;

% trajectory is on yz plane
% from xy to yz
% x=0, y=y, z=x

function pos = pos_from_angle(r, a)
    pos = [0 ; r*sin(a); r*cos(a)];
end

function vel = get_vel(r, angle_start, angle_end, T, t)
    angle1 = tj_from_line(angle_start, angle_end, T, t);
    pos1 = pos_from_angle(r, angle1);
    angle2 = tj_from_line(angle_start, angle_end, T, t+dt);
    pos2 = pos_from_angle(r, angle2);
    vel = (pos2 - pos1)/dt;
end

if t==0
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
% Track the letter "T"
% 1:1:1
elseif t < T_t*1/3
    pos_start = [0;0;0];
    pos_end = [0;7;0];
    t_tl = T_t*1/3;
    t_c = t-0;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
elseif t < T_t*2/3
    pos_start = [0;7;0];
    pos_end = [0;7;-2];
    t_tl = T_t/3;
    t_c = t-T_t*1/3;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Connection
elseif t < T_t
    pos_start = [0;7;-2];
    pos_end = [0;7;3];
    t_tl = T_t/3;
    t_c = t-T_t*2/3;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Track the letter "U"
% 2:3:2:2
% Straight1
elseif t - T_t < T_u*2/9
    pos_start = [0;7;3];
    pos_end = [0;2;3];
    t_tl = T_u*2/9;
    t_c = t - T_t;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Semicirle with radius R_u
elseif t - T_t < T_u*5/9
    angle_start = pi;
    angle_end = 2*pi;
    center = [0;2;5];
    t_tl = T_u*3/9;
    t_c = t - T_t - T_u*2/9;
    angle = tj_from_line(angle_start, angle_end, t_tl, t_c);
    pos = pos_from_angle(R_u, angle) + center;
    vel = get_vel(R_u, angle_start, angle_end, t_tl, t_c);
    vel_tdt = get_vel(R_u, angle_start, angle_end, t_tl, t_c+dt);
    acc = (vel_tdt - vel)/dt;
% Straight2
elseif t - T_t < T_u*7/9
    pos_start = [0;2;7];
    pos_end = [0;7;7];
    t_tl = T_u*2/9;
    t_c = t - T_t - T_u*5/9;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Connection
elseif t - T_t < T_u
    pos_start = [0;7;7];
    pos_end = [0;7;8.5];
    t_tl = T_u*2/9;
    t_c = t - T_t - T_u*7/9;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Track the letter "D"
% 5:2:3
% Semicircle with radium R_d
elseif t - T_t - T_u < T_d*5/10
    angle_start = pi/2;
    angle_end = - pi/2;
    center = [0;3.5;8.5];
    t_tl = T_d*5/10;
    t_c = t - T_t - T_u;
    angle = tj_from_line(angle_start, angle_end, t_tl, t_c);
    pos = pos_from_angle(R_d, angle) + center;
    vel = get_vel(R_d, angle_start, angle_end, t_tl, t_c);
    vel_tdt = get_vel(R_d, angle_start, angle_end, t_tl, t_c+dt);
    acc = (vel_tdt - vel)/dt;
% Straight2
elseif t - T_t - T_u < T_d*7/10
    pos_start = [0;0;8.5];
    pos_end = [0;0;8];
    t_tl = T_d*2/10;
    t_c = t - T_t - T_u - T_d*5/10;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
% Straight3
elseif t - T_t - T_u < T_d
    pos_start = [0;0;8];
    pos_end = [0;7;8];
    t_tl = T_d*3/10;
    t_c = t - T_t - T_u - T_d*7/10;
    [pos, vel, acc] = tj_from_line(pos_start, pos_end, t_tl, t_c);
else
    pos = [0;7;8];
    vel = [0;0;0];
    acc = [0;0;0];
end

yaw = 0;
yawdot = 0;
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
