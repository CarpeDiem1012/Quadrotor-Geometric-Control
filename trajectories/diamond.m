function [desired_state] = diamond(t)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the


% You have to set the pos, vel, acc, yaw and yawdot variables
T = 12; % total time you expect to finish the task

if t==0
    % position you return for t == 0
    pos = [0;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
elseif t < T/4
    start_pos = [0;0;0];
    end_pos = [0.25;sqrt(2);sqrt(2)];
    [pos, vel, acc] = tj_from_line(start_pos,end_pos,T/4,t-0);
elseif t < T/2
    start_pos = [0.25;sqrt(2);sqrt(2)];
    end_pos = [0.5;0;2*sqrt(2)];
    [pos, vel, acc]  = tj_from_line(start_pos,end_pos,T/4,t-T/4);
elseif t < 3*T/4
    start_pos = [0.5;0;2*sqrt(2)];
    end_pos = [0.75;-sqrt(2);sqrt(2)];
    [pos, vel, acc]  = tj_from_line(start_pos,end_pos,T/4,t-T/2);
elseif t < T
    start_pos = [0.75;-sqrt(2);sqrt(2)];
    end_pos = [1;0;0];
    [pos, vel, acc]  = tj_from_line(start_pos,end_pos,T/4,t-3*T/4);
else
    pos = [1;0;0];
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
