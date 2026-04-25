function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

estimated_pose = public_vars.mu;
estimated_pose(3) = mod(estimated_pose(3),2*pi());

end

