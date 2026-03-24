function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

%%
persistent tmp_lidar_data;
%Optional step for data gathering... Probably will be commented out later
data_acq_periods = 100;
if(read_only_vars.counter <= data_acq_periods)
    tmp_lidar_data(read_only_vars.counter,:) = read_only_vars.lidar_distances;
    
     if(read_only_vars.counter == data_acq_periods)
         lidar_deviation = std(tmp_lidar_data)
         gnss_deviation = std(read_only_vars.gnss_history)
         figure(42)
         subplot(2,1,1)
         hold on
         for i = 1:8
            histogram(tmp_lidar_data(:,i),'FaceAlpha',0.5);
         end
         hold off
         subplot(2,1,2)
         hold on
         for i = 1:2
            histogram(read_only_vars.gnss_history(:,i),-1:0.1:3,'FaceAlpha',0.5);
         end
         hold off
     end

end


%%
% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

