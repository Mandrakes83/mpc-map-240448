function [public_vars] = init_particle_filter(read_only_vars, public_vars,particle_count)
%INIT_PARTICLE_FILTER Summary of this function goes here

if nargin < 3
particle_count = 1000;
end

random_vector = rand(particle_count,3);

random_vector(:,1) = (random_vector(:,1)*read_only_vars.map.limits(3)) - read_only_vars.map.limits(1); 
random_vector(:,2) = (random_vector(:,2)*read_only_vars.map.limits(4)) - read_only_vars.map.limits(2);
random_vector(:,3) = random_vector(:,3)*2*pi();

public_vars.particles = random_vector;

end

