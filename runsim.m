% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** RO47001 QUADROTOR SIMULATION *****************
close all
clear all
addpath('utils')
addpath('trajectories')

% You can change trajectory here

% trajectory generator
% trajhandle = @hover;
% trajhandle = @circle;
% trajhandle = @diamond;
trajhandle = @tudtrj;

% controller
controlhandle = @controller;

% real-time 
real_time = true;

% max time, definitely it can be shorter
time_tol = 1000;

% parameters for simulation
params = crazyflie();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0);
des_stop  = trajhandle(inf);
stop      = des_stop.pos;
x0        = init_state( des_start.pos, 0 );
xtraj     = zeros(max_iter*nstep, length(x0));
ttraj     = zeros(max_iter*nstep, 1);


x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
OUTPUT_TO_VIDEO = 1;
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('diamond');
    open(v)
end

fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter
    iter;
    timeint = time:tstep:time+cstep;

    tic;
    
    % Initialize quad plot
    if iter == 1
        QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
        desired_state = trajhandle(time);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end
    
    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, 1, controlhandle, trajhandle, params), timeint, x);
    x    = xsave(end, :)';
    
    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    % Update quad plot
    desired_state = trajhandle(time + cstep);
    QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    if OUTPUT_TO_VIDEO == 1
        im = frame2im(getframe(gcf));
        writeVideo(v,im);
    end
    
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
end

if OUTPUT_TO_VIDEO == 1
    close(v);
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj

xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);


% Plot the saved position and velocity of each robot

% Truncate saved variables
QP.TruncateHist();
% Plot position for each quad
h_pos = figure('Name', ' position');
plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% Plot velocity for each quad
h_vel = figure('Name', 'velocity');
plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

xtraj_des = zeros(length(ttraj),3);
for i = 1:length(ttraj)
    state_des = trajhandle(ttraj(i));
    xtraj_des(i,:) = state_des.pos(:);
end

fprintf('finished.\n')
error_pos = sum(sum((xtraj(:,1:3)- xtraj_des).^2))*tstep;
energy = sum(sum(xtraj(11:13,:).^2))*tstep;
fprintf("\nThe Total Time is: %4.2f s \n",time);
fprintf("The Sum of Squared Error for Position over Total Time is: %4.5f m \n", error_pos);
fprintf("The Battery Storage Spent over Total Time is: %4.5f \n", energy);

