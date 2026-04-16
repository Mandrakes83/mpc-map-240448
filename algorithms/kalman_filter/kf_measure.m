function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here

K = sigma * kf.C' * inv(kf.C * sigma * kf.C' + kf.R);

new_mu = mu' + K*(z' - kf.C*(mu'));
new_sigma = (eye(3) - K*kf.C)*sigma;

end

