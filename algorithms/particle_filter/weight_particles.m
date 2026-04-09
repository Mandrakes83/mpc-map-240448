function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

lidar_matrix = ones(size(particle_measurements,1),1)*lidar_distances;

tmp = vecnorm(particle_measurements-lidar_matrix,2,2);

tmp(tmp == 0) = 0.001;

weights = 1./tmp;

weights = weights/sum(weights);

end

