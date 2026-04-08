function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here

persistent waypoint_counter;

target_acquired_distance = 0.4;

if(isempty(waypoint_counter))
    waypoint_counter = 1;
end

target = path(waypoint_counter,:);

if norm(estimated_pose(1:2) - target) < target_acquired_distance && waypoint_counter < size(path,1)
    waypoint_counter = waypoint_counter+1;
    target = path(waypoint_counter,:);
end

