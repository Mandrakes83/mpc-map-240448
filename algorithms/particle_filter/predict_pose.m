function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

l = read_only_vars.agent_drive.interwheel_dist;
Vr = motion_vector(1);
Vl = motion_vector(2);
theta = old_pose(3);
delta_t = read_only_vars.sampling_period;

if(Vr ~= Vl)
    R = (l/2) * (Vl + Vr)/(Vr - Vl);
else
    R = 0;
end
omega = (Vr - Vl)/l;

x = old_pose(1);
y = old_pose(2);

ICCx = x - R*sin(theta);
ICCy = y + R*cos(theta);

Part1 = [cos(omega*delta_t),-sin(omega*delta_t),0; 
         sin(omega*delta_t), cos(omega*delta_t),0;
         0,                  0,                 1];

Part2 = [x-ICCx;
         y-ICCy;
         theta];

Part3 = [ICCx;
         ICCy;
         omega*delta_t];


new_pose = (Part1*Part2) + Part3;
end

