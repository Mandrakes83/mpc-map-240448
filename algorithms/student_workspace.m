function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

Init_cycle_count = 51;

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
if (read_only_vars.counter > Init_cycle_count)
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars)'; % (x,y,theta)

end
% public_vars.estimated_pose = read_only_vars.mocap_pose; % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

public_vars.path(1:100,:) = [linspace(2,10,100);linspace(2,8,100)]';
public_vars.path(101:150,:) = [linspace(10,16,50);linspace(8,6,50)]';
public_vars.path(151:200,:) = [linspace(16,16,50);linspace(6,2,50)]';




% 13. Plan next motion command
if (read_only_vars.counter > Init_cycle_count)
    public_vars = plan_motion(read_only_vars, public_vars);

elseif(read_only_vars.counter == Init_cycle_count-1)
    % Process measured data
    public_vars.estimated_pose(1:2) = mean(read_only_vars.gnss_history);
    public_vars.estimated_pose(3) = pi();

    public_vars.mu = public_vars.estimated_pose;

    public_vars.sigma(1,1) = std(read_only_vars.gnss_history(:,1))^2;
    public_vars.sigma(2,2) = std(read_only_vars.gnss_history(:,2))^2;
    public_vars.sigma(3,3) = (30*pi/180)^2;

    
    public_vars.kf.Q = diag([
            0.02^2;                 % x process noise
            0.02^2;                 % y process noise
            (2*pi/180)^2            % theta process noise
        ]);

    public_vars.kf.R = cov(read_only_vars.gnss_history);
   

else
    % Wait for data to measure

end

end
