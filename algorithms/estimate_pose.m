function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here
if(public_vars.kf_enabled)
    estimated_pose = public_vars.mu';

elseif (public_vars.pf_enabled)
    estimated_pose = median(public_vars.particles);

end

estimated_pose(3) = mod(estimated_pose(3),2*pi());
end

