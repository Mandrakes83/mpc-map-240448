function [estimated_pose] = estimate_pose(public_vars,pf_weights)
%ESTIMATE_POSE Summary of this function goes here
if(public_vars.kf_enabled)
    estimated_pose = public_vars.mu';

elseif (public_vars.pf_enabled)
    
    particles = public_vars.particles;
    weights   = pf_weights;
    
    weights = weights / sum(weights);
    
    % --- Median for position ---
    x_est = weighted_median(particles(:,1), weights);
    y_est = weighted_median(particles(:,2), weights);
    
    % --- Circular median for orientation ---
    angles = particles(:,3);
    cost = zeros(size(angles));
    
    for i = 1:length(angles)
        diff = atan2(sin(angles - angles(i)), cos(angles - angles(i)));
        cost(i) = sum(abs(diff) .* weights);
    end
    
    [~, idx] = min(cost);
    theta_est = angles(idx);
    
    estimated_pose = [x_est, y_est, theta_est];

end

estimated_pose(3) = mod(estimated_pose(3),2*pi());
end

function m = weighted_median(values, weights)
    [values, idx] = sort(values);
    weights = weights(idx);
    w_cum = cumsum(weights) / sum(weights);
    m = values(find(w_cum >= 0.5, 1));
end
