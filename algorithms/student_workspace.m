function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

persistent Localization_state;
Init_cycle_count = 30;

persistent Init_stop;
KALMAN = 1;
PF = 2;
KALMAN_BOOT = 3;
PF_BOOT = 4;

% --- State initialization ---
if isempty(Localization_state)
    if any(isnan(read_only_vars.gnss_position))
        Localization_state = PF_BOOT;
    else
        Localization_state = KALMAN_BOOT;
    end
end

% --- State machine ---
switch Localization_state

    % =========================
    case KALMAN_BOOT
    % =========================
    persistent kalman_transfer;
    if(isempty(kalman_transfer))
        kalman_transfer = 0;
    end

        if isempty(Init_stop)
            Init_stop = read_only_vars.counter + Init_cycle_count;
        end
        public_vars.motion_vector = [0,0];
        % Collect data during init cycles
        if read_only_vars.counter == Init_stop - 1
            public_vars = init_kalman_filter(read_only_vars, public_vars);
            % GNSS-based initialization
            public_vars.estimated_pose(1:2) = mean(read_only_vars.gnss_history(Init_stop - Init_cycle_count:end,:));
            
            if(kalman_transfer == 0)
                public_vars.estimated_pose(3) = pi;
            else
                public_vars.estimated_pose(3) = public_vars.estimated_pose(3);
            end
            kalman_transfer = 0;
            public_vars.mu = public_vars.estimated_pose;

            public_vars.sigma = zeros(3);
            public_vars.sigma(1,1) = std(read_only_vars.gnss_history(Init_stop - Init_cycle_count:end,1))^2;
            public_vars.sigma(2,2) = std(read_only_vars.gnss_history(Init_stop - Init_cycle_count:end,2))^2;

            if(kalman_transfer == 0)
                public_vars.sigma(3,3) = pi^2;
            else
                public_vars.sigma(3,3) = ((pi/6)^2)*3;
            end
            

            public_vars.kf.Q = diag([
                0.005^2;
                0.01^2;
                0.025^2
            ]);

            public_vars.kf.R = cov(read_only_vars.gnss_history(Init_stop - Init_cycle_count:end,:));

        elseif read_only_vars.counter >= Init_stop
            Localization_state = KALMAN;
            public_vars.kf_enabled = 1;
            Init_stop = []; %enable reinit later on
        end


    % =========================
    case PF_BOOT
    % =========================

        
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.pf_enabled = 1;
        Localization_state = PF;            

    % =========================
    case KALMAN
    % =========================
        persistent stabilization_counter;
        if(isempty(stabilization_counter))
            stabilization_counter = 0;
        end

        if any(isnan(read_only_vars.gnss_position))
            Localization_state = PF_BOOT;
            public_vars.kf_enabled = 0;
            stabilization_counter = [];
            return;
        end

        % Update Kalman filter
        [public_vars.mu, public_vars.sigma] = ...
            update_kalman_filter(read_only_vars, public_vars);

        % Estimate pose
        public_vars.estimated_pose = estimate_pose(public_vars);

        % Path planning
        public_vars.path = plan_path(read_only_vars, public_vars);
        if(stabilization_counter < 20)
            stabilization_counter = stabilization_counter + 1;
            public_vars.motion_vector = [0,0];
        else
        % Motion planning
            public_vars = plan_motion(read_only_vars, public_vars);
        end
        


    % =========================
    case PF
    % =========================
    persistent init_counter;
    if(isempty(init_counter))
        init_counter = 0;
    end

    persistent quit_counter;
    if(isempty(quit_counter))
        quit_counter = 0;
    end    

    if ~any(isnan(read_only_vars.gnss_position))
        quit_counter = quit_counter + 1;
        
    else
        quit_counter = 0;
    end

    if(quit_counter > 50)
        Localization_state = KALMAN_BOOT;
        public_vars.pf_enabled = 0;
        init_counter = [];
        quit_counter = [];
        kalman_transfer = 1;
        return;
    end
 

    % Update particle filter
    if public_vars.pf_enabled
        [public_vars.particles,pf_weights] = ...
            update_particle_filter(read_only_vars, public_vars);
    end

    
    if(init_counter < 30)
        public_vars.motion_vector = [-0.2,0.2];
        init_counter = init_counter +1;
    elseif(init_counter < 60)
        public_vars.motion_vector = [0.2,0.2];
        init_counter = init_counter +1;
    else
        % Estimate pose
        public_vars.estimated_pose = estimate_pose(public_vars,pf_weights);
        % Path planning
        public_vars.path = plan_path(read_only_vars, public_vars);
        % Motion planning
        public_vars = plan_motion(read_only_vars, public_vars);
        
    end

end
end