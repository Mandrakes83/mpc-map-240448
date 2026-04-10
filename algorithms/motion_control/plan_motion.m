function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

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

if(v < 0.2 && w < 0.1)
    v = 0.2;
end

public_vars.motion_vector = [(2*v+w)/2,(2*v-w)/2]; %right,left
end