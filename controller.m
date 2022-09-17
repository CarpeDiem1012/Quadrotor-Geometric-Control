function [F, M, trpy, drpy] = controller(qd, t, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

%% =================== Your code goes here ===================
% 
% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

phi_des   = 0;
theta_des = 0;
psi_des   = 0;
%
pos = qd.pos;
vel = qd.vel;
roll = qd.euler(1);
pitch = qd.euler(2);
yaw = qd.euler(3);
omega = qd.omega;
%
pos_des = qd.pos_des;
vel_des = qd.vel_des;
acc_des = qd.acc_des;
%
roll_des = phi_des;
rolldot_des = 0;
pitch_des = theta_des;
pitchdot_des = 0;
yaw_des = qd.yaw_des;
yawdot_des = qd.yawdot_des;
%
I = params.I;              % moment of inertia
grav =  params.grav;     % gravitational constant g (9.8...m/s^2)
mass =  params.mass;     % mass of robot
%
u    = zeros(4,1); % control input u, you should fill this in

% According to the linearized controller aroung the equilibrium point
% Kd = [10;10;10];
% Kp = [100;100;100];
% acc_command = acc_des + Kd.*(vel_des-vel) + Kp.*(pos_des-pos);
% u(1) = mass*(grav + acc_command(3));
% 
% Kd_angle = [100;100;100];
% Kp_angle = [2000;2000;2000];
% roll_des = (acc_command(1)*sin(yaw_des) - acc_command(2)*cos(yaw_des))/grav;
% pitch_des = (acc_command(1)*cos(yaw_des) - acc_command(2)*sin(yaw_des))/grav;
% error_omega = [rolldot_des - omega(1);pitchdot_des - omega(2);yawdot_des - omega(3)];
% error_angle = [roll_des - roll;pitch_des - pitch;yaw_des - yaw];
% omegadot = Kd_angle.*error_omega + Kp_angle.*error_angle;
% u(2:4) = I*omegadot + cross(omega,I*omega);

%
% According to the geometric nonlinear controller
Kd = [20;20;40];
Kp = [100;100;200];
acc_command = acc_des + Kd.*(vel_des-vel) + Kp.*(pos_des-pos);
%
% 1. Get the ideal but unreachabel external force F
% The suffix "ideal" means being strictly as required but unreachable due
% to geometric constraints. The suffix "des" means being not strictly as
% required but practically reachable under geometric constaints, which is
% eventually what we want.
F_ideal = [0;0;mass*grav] + mass*acc_command;
%
% 2. Get FixBodyFrame RotationMatrix with Z-X-Y(psi-phi-theta)
R = eulzxy2rotmat([roll,pitch,yaw]);
%
% 3. Get the desired thrust by projection (try its best to compromise F_ideal)
u(1) = R(:,3).' * F_ideal;
%
% 4. Get the desired FixBodyFrame RotationMatrix by CrossProduct
%   4.1 Get desired Z axis
    Z_axis_des = F_ideal/sqrt(F_ideal.'*F_ideal);
%   4.2 Get ideal but unreachable X axis (not orthogonal to Z_axis_des)
    R_ideal = eulzxy2rotmat([roll_des,pitch_des,yaw_des]);
    X_axis_ideal = R_ideal(:,1);
%   4.3 Get desired Y axis by CrossProduct
    cross_ZX = cross(Z_axis_des, X_axis_ideal);
    Y_axis_des = cross_ZX/sqrt(cross_ZX.'*cross_ZX);
%   4.4 Get desired X axis (orthogonal to Z_axis_des)
    X_axis_des = cross(Y_axis_des, Z_axis_des);
% Get the desired RotationMatrix and desired euler angles
R_des = [X_axis_des, Y_axis_des, Z_axis_des];
eul_des = rotmat2eulzxy(R_des);
phi_des = eul_des(1); theta_des = eul_des(2); psi_des = eul_des(3);
%
% 5. Get the RotationError and desired moment
error_rotation_skew = (R_des.'*R - R.'*R_des);
error_rotation = (veemap(error_rotation_skew)/2).';
omega_des = [rolldot_des; pitchdot_des; yawdot_des];
error_omega = omega - omega_des;
%
Kw = [200;200;100];
Kr = [3000;3000;1500];
% PD controller for the moment
% u(2:4) = I*(-Kw.*error_omega-Kr.*error_rotation) + cross(omega,I*omega);
u(2:4) = I*(-Kw.*error_omega-Kr.*error_rotation) + cross(omega,I*omega)...
        - I*(cross(omega,R.'*R_des*omega) - R.'*R_des*omega_des);

% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================
%%
% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
