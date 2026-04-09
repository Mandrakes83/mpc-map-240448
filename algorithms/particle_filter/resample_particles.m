function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here

N = length(particles);

w_max = max(weights);

index = randi([1,N]);
new_particles = particles;

for i = 1:N
    beta = rand(1)*2*w_max;

    while(weights(index) < beta)
        beta = beta - weights(index);
        index = mod(index,N)+1;
    end

    new_particles(i,:) = particles(index,:);
end


end

