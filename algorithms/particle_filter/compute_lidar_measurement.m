function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

persistent counter;
if(isempty(counter))
    counter = 1;
else
    counter = counter +1;
end

measurement = zeros(1, length(lidar_config));

for i = 1:size(length(lidar_config))
   intersections = ray_cast(pose(1:2),map.walls,mod(lidar_config(1)+pose(3),2*pi()));
end

measurement = min(vecnorm(intersections,2,2));

end

