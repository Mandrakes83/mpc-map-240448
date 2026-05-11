function [public_vars] = init_particle_filter(read_only_vars, public_vars,particle_count)
%INIT_PARTICLE_FILTER Summary of this function goes here

if nargin < 3
particle_count = 500;
end

if nargin == 3
    known_pose = [];
else
    known_pose = public_vars.estimated_pose;
end

if(isempty(known_pose))
random_vector = rand(particle_count,3);

random_vector(:,1) = (random_vector(:,1)*read_only_vars.map.limits(3)) - read_only_vars.map.limits(1); 
random_vector(:,2) = (random_vector(:,2)*read_only_vars.map.limits(4)) - read_only_vars.map.limits(2);
random_vector(:,3) = random_vector(:,3)*2*pi();

public_vars.particles = random_vector;
else


mu    = read_only_vars.est_position_history(end-5,:);       % [1x3]
Sigma = public_vars.sigma;                    % [3x3]

% ---- Safety: ensure covariance is valid ----
Sigma = (Sigma + Sigma.') / 2;    % enforce symmetry
Sigma = Sigma + 1e-9 * eye(3);    % numerical stability

% ---- Cholesky sampling (NO TOOLBOX) ----
L = chol(Sigma, 'lower');         % Sigma = L*L'
Z = randn(particle_count,3);      % standard normal

particles = Z * L.' + mu;

% ---- Angle wrapping ----
particles(:,3) = mod(particles(:,3) + pi, 2*pi) - pi;

% ---- Keep particles inside map ----
particles(:,1) = min(max(particles(:,1), read_only_vars.map.limits(1)), read_only_vars.map.limits(3));
particles(:,2) = min(max(particles(:,2), read_only_vars.map.limits(2)), read_only_vars.map.limits(4));

public_vars.particles = particles;


end


end

