function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

L = read_only_vars.agent_drive.interwheel_dist;

%Spocitat polohu P
theta = public_vars.estimated_pose(3);
epsilon = 0.05; %delka tycky
xp = public_vars.estimated_pose(1) + epsilon*cos(theta);
yp = public_vars.estimated_pose(2) + epsilon*sin(theta);
P_pose = [xp,yp];

kappa = norm(target - P_pose);

dxp = kappa*(target(1) - xp);
dyp = kappa*(target(2) - yp);

v = dxp*cos(theta) + dyp*sin(theta);
w = (-dxp*sin(theta) + dyp*cos(theta))/(epsilon);

% Limits
v_max = 0.9; 
v_min = 0.3;
w_max = pi/16;
w_deadzone = pi/16;


% Limitacia uhlovej rychlosti robota
if w < 0
    w = max(w,-w_max);
elseif(w > 0)
    w = min(w,w_max);
end

% Minimalna rychlost robota
if(abs(w) < w_deadzone) 
    % Ak ide rovno exstuje min speed
    v = max(min(v, v_max), v_min);
else
    % Ak chce rotovat existuje len max speed
    v = min(v,v_max);
end


public_vars.motion_vector = [(2*v+w)/2,(2*v-w)/2]; %right,left
end