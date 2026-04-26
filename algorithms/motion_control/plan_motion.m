function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

persistent v_r_memory;
persistent v_l_memory;

if(and(isempty(v_r_memory),isempty(v_l_memory)))
   v_r_memory = 0;
   v_l_memory = 0;
end

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

L = read_only_vars.agent_drive.interwheel_dist;

%Spocitat polohu P
theta = public_vars.estimated_pose(3);
epsilon = 0.15; %delka tycky
xp = public_vars.estimated_pose(1) + epsilon*cos(theta);
yp = public_vars.estimated_pose(2) + epsilon*sin(theta);
P_pose = [xp,yp];

kappa = norm(target - P_pose);

delta_t = read_only_vars.sampling_period;

dxp = kappa*(target(1) - xp)*delta_t;
dyp = kappa*(target(2) - yp)*delta_t;

v = dxp*cos(theta) + dyp*sin(theta);
w = (-dxp*sin(theta) + dyp*cos(theta))/(L);

% Limits
v_max = 0.75          * delta_t; 
v_min = 0.25          * delta_t;
w_max = pi/4          * delta_t;
w_deadzone = pi/32    * delta_t;


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


wheel_speed_slew_rate = 0.05;

v_r = (2*v+w*L)/(2*delta_t);
v_l = (2*v-w*L)/(2*delta_t);

if(abs(v_r) - abs(v_r_memory) > wheel_speed_slew_rate)
    if(v_r > v_r_memory)
        v_r = v_r + wheel_speed_slew_rate;
    else
        v_r = v_r - wheel_speed_slew_rate;
    end
end

if(abs(v_l) - abs(v_l_memory) > wheel_speed_slew_rate)
    if(v_l > v_l_memory)
        v_l = v_l + wheel_speed_slew_rate;
    else
        v_l = v_l - wheel_speed_slew_rate;
    end
end

public_vars.motion_vector = [v_r,v_l] %right,left
end