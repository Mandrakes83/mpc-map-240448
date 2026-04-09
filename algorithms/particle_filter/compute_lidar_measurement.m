function [measurements] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurements = zeros(1, length(lidar_config));

for i = 1:length(lidar_config)
   intersections = ray_cast(pose(1:2),map.walls,lidar_config(i)+pose(3));

   tmp = ones(size(intersections,1),1)*pose(1:2);

   measurements(i) = min(vecnorm((intersections-tmp),2,2));
end

measurements(isnan(measurements)) = +Inf;

end

