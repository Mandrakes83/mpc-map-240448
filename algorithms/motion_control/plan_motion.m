function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

if read_only_vars.counter < 50
    public_vars.motion_vector = [1, 1];        % [Right, Left]

elseif read_only_vars.counter < 115
    public_vars.motion_vector = [0.9, 1];      % [Right, Left]

elseif read_only_vars.counter < 150
    public_vars.motion_vector = [1, 1];        % [Right, Left]

elseif read_only_vars.counter < 205
    public_vars.motion_vector = [1, 0.9];

elseif read_only_vars.counter < 300
    public_vars.motion_vector = [1, 1];

else
    public_vars.motion_vector = [0, 0];        % [Right, Left]
end




end