function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here




if(isempty(public_vars.estimated_pose))
    planning_required = 0;
else
    planning_required = 1;
end

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    
    path = public_vars.path;
    
end

end

