function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here

l = read_only_vars.agent_drive.interwheel_dist;
Vr = motion_vector(1);
Vl = motion_vector(2);
delta_t = read_only_vars.sampling_period;


x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);


if(abs(Vr-Vl) > 0.0001)
    R = (l/2) * (Vl + Vr)/(Vr - Vl);
    omega = (Vr - Vl)/l;

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

else
    R = 0;
    v = (Vr + Vl) / 2;
    new_pose(1,1) = x + v * delta_t * cos(theta);
    new_pose(2,1) = y + v * delta_t * sin(theta);
    new_pose(3,1) = theta;
end

dist = norm([old_pose(1)-new_pose(1) old_pose(2)-new_pose(2)]);
dturn = abs(old_pose(3) - new_pose(3));     

sigma_x     = 0.05 * dist + 0.001; 
sigma_y     = 0.05 * dist + 0.001;
sigma_theta = 0.05 * dturn + 0.0005;

new_pose = new_pose' + randn(1,3) .* [sigma_x, sigma_y, sigma_theta];

new_pose(3) = atan2(sin(new_pose(3)), cos(new_pose(3)));

offset_for_walls = 0.1;
if(new_pose(1) < read_only_vars.map.limits(1))
    new_pose(1) = read_only_vars.map.limits(1) + offset_for_walls;
elseif(new_pose(1) > read_only_vars.map.limits(3))
    new_pose(1) = read_only_vars.map.limits(3) - offset_for_walls;
end

if(new_pose(2) < read_only_vars.map.limits(2))
    new_pose(2) = read_only_vars.map.limits(2) + offset_for_walls;
elseif(new_pose(2) > read_only_vars.map.limits(4))
    new_pose(2) = read_only_vars.map.limits(4) - offset_for_walls;
end

end

