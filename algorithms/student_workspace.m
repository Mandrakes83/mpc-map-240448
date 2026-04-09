function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

%%
% persistent tmp_lidar_data;
% %Optional step for data gathering... Probably will be commented out later
% data_acq_periods = 500;
% if(read_only_vars.counter <= data_acq_periods)
%     tmp_lidar_data(read_only_vars.counter,:) = read_only_vars.lidar_distances;
% 
%      if(read_only_vars.counter == data_acq_periods)
%          lidar_deviation = std(tmp_lidar_data)
%          gnss_deviation = std(read_only_vars.gnss_history)
%          figure(42)
%          subplot(2,1,1)
%          hold on
%          for i = 1:8
%             histogram(tmp_lidar_data(:,i),'FaceAlpha',0.5);
%          end
%          legend("Kanal 1", "Kanal 2","Kanal 3","Kanal 4","Kanal 5","Kanal 6","Kanal 7","Kanal 8");
%          ylabel("Počet [-]")
%          xlabel("Vzdialenosť [m]")
%          hold off
%          subplot(2,1,2)
%          hold on
%          for i = 1:2
%             histogram(read_only_vars.gnss_history(:,i),'FaceAlpha',0.5);
%          end
%          legend("Kanal 1", "Kanal 2");
%          ylabel("Počet [-]")
%          xlabel("Poloha [m]")
%          hold off
% 
%          lidar_cov = cov(tmp_lidar_data)
%          gnss_cov = cov(read_only_vars.gnss_history)
% 
%          lidar_pdf = norm_pdf(-2:0.1:2,0,lidar_deviation(1))
%          gnss_pdf = norm_pdf(-2:0.1:2,0,gnss_deviation(1))
% 
%          figure(43)
%          hold on
%          plot(-2:0.1:2,lidar_pdf,'-x')
%          plot(-2:0.1:2,gnss_pdf,'-+')
%          hold off
%          legend("Lidar","Gnss")
%          xlabel("Vzdialenosť / Poloha [m]")
%          ylabel("Pravdepodobnosť [-]")
% 
%      end
% 
% end


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
% public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
public_vars.estimated_pose = read_only_vars.mocap_pose; % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

path_select = 0;
pocet_bodov = 50;

if(path_select == 0)
    public_vars.path = [[5,5];[10,10];[13,13];[17.5,13];[19,19]];
elseif (path_select == 1)
    body = [1 1; 12 5; 19 19];
    data_pre_kruh = triangulation([1 2 3], body(:,1), body(:,2));
    [stred,polomer] = circumcenter(data_pre_kruh);
    uhol = linspace(1.51*pi,1.99*pi,pocet_bodov);
    
    cesta_x = stred(1) + polomer*cos(uhol);
    cesta_y = stred(2) + polomer*sin(uhol);

    public_vars.path = [cesta_x',cesta_y'];
else
   offset = 18.5;
   body = linspace(1,19,pocet_bodov);
   path = sin(body);
   public_vars.path = [body',(path+offset)'];
   
   public_vars.path = [1, 1;
                       1, 2.5;
                       1, 5;
                       1, 7.5;
                       1, 10;
                       1, 12.5;
                       1, 18.5;
                       body', (path + offset)'];
end



% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end
