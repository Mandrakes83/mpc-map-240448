function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Iterative path smoothing (gradient descent)
% old_path: Nx2 matrix [x y]
% new_path: Nx2 smoothed path

% Parameters (tune these)
alpha = 0.6;      % fidelity to original path
beta  = 0.4;      % smoothness weight
tol   = 1e-5;     % convergence tolerance
max_iter = 1000;  % safety bound

% Original path (X)
X = old_path;

% Smoothed path (Y) – start with copy
Y = X;

N = size(X,1);

iter = 0;
change = inf;

while change > tol && iter < max_iter
    change = 0;
    iter = iter + 1;

    % Do NOT move start and goal
    for i = 2:N-1
        y_old = Y(i,:);

        Y(i,:) = Y(i,:) ...
            + alpha * (X(i,:) - Y(i,:)) ...
            + beta  * (Y(i-1,:) + Y(i+1,:) - 2*Y(i,:));

        change = change + norm(Y(i,:) - y_old);
    end
end

new_path = Y;

end