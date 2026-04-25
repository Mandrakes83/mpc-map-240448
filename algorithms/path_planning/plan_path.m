function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here



if(isempty(public_vars.path))
    planning_required = 1;
else 
    planning_required = 0;
end

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    
    path = public_vars.path;
    
end

end

