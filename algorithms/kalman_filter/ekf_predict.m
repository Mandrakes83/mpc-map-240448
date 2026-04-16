function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here

x = mu(1);
y = mu(2);
theta = mu(3);


new_mu(1) = x + cos(theta)*u(1)*sampling_period;
new_mu(2) = y + sin(theta)*u(1)*sampling_period;
new_mu(3) = theta + u(2)*sampling_period;

G = [1 0 -sin(theta)*u(1)*sampling_period;
     0 1  cos(theta)*u(1)*sampling_period;
     0 0  1                               ];

new_sigma = G*sigma*(G') + kf.Q;

end

