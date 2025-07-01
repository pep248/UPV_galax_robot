% Initialize simulation environment
clc
clear all
close all
warning('off', 'all')  % Disable all warnings
addpath(genpath(pwd))  % Adds all scripts in the current directory to the path


%% 

% === Load or Create Agent ===
file_name = 'Agent77000.mat';

if exist(file_name, 'file') == 2
    fileVars = who('-file', file_name);
    global agentObj;
    if ismember('agentObj', fileVars)
        load(file_name, 'agentObj');
        disp("Agent loaded successfully.");
    elseif all(ismember({'saved_agent', 'savedAgentResult'}, fileVars))
        load(file_name, 'saved_agent', 'saved_agent_result');
        agentObj = saved_agent;
        disp("Agent loaded from saved_agent.");
    else
        error('trainedAgent.mat does not contain a recognizable agent variable.');
    end
end


%% 
% 
SetupFcn();

global robot
robot.sim_params.visualization_tools = true;
robot.sim_params.use_parallel = false;




%% 
% 
% Reset simulation again to clear any residuals
ResetFunction();

numEpisodes = 10;
maxSteps = 600;
episodeRewards = zeros(numEpisodes,1);

for ep = 1:numEpisodes
    get_new_goal(); % Add this line to randomize environment
    [obs, ~] = ResetFunction();
    totalReward = 0;
    isDone = false;
    step = 1;
    while ~isDone && step <= maxSteps
        action = getAction(agentObj, obs);
        action = action{1};
        [nextObs, reward, isDone, ~] = StepFunction(action, []);
        totalReward = totalReward + reward;
        obs = nextObs;
        step = step + 1;
    end
    episodeRewards(ep) = totalReward;
    fprintf('Test Episode %d: Total Reward = %.2f, Steps = %d\n', ep, totalReward, step-1);
end
fprintf('Average Test Reward over %d episodes: %.2f\n', numEpisodes, mean(episodeRewards));




%% 
% 

% Simulation step function
function [NextObs, Reward, IsDone, LoggedSignals] = StepFunction(Action, LoggedSignals)
    global robot
    
    % Update DWA parameters based on Action
    robot.DWA.alpha = Action(1);
    robot.DWA.beta = Action(2);
    robot.DWA.gamma = Action(3);
    robot.DWA.delta = Action(4);

    if robot.sim_params.visualization_tools == true
        robot.sim_params.actions.alpha = [robot.sim_params.actions.alpha ; robot.DWA.alpha];
        robot.sim_params.actions.beta = [robot.sim_params.actions.beta ; robot.DWA.beta];
        robot.sim_params.actions.gamma = [robot.sim_params.actions.gamma ; robot.DWA.gamma];
        robot.sim_params.actions.delta = [robot.sim_params.actions.delta ; robot.DWA.delta];
    end
    
    % Update lidar
    % [robot.lidar_points_xy, robot.lidar_points_r] = TranslateLidar(robot)
    % robot.lidar_points_angles = [robot.lidar(robot.pose), robot.lidar.scanAngles];

    
    [v_opt, omega_opt] = CalculateDWA(robot);
    robot.velocity = [v_opt, omega_opt];

    velB = [v_opt;0;omega_opt];          % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,robot.pose);  % Convert from body to world
    
    % Calculate new pose
    robot.previous_pose = robot.pose;
    robot.pose = robot.pose + vel'*robot.sim_params.sim_T;
    % Store pose in array
    robot.sim_params.pose_array(:,robot.sim_params.iteration) = robot.pose;
    
    
    % Check if the new pose is within the map boundaries with tolerance
    tolerance = 0.3;
    x_min = robot.map.XWorldLimits(1) + tolerance;
    x_max = robot.map.XWorldLimits(2) - tolerance;
    y_min = robot.map.YWorldLimits(1) + tolerance;
    y_max = robot.map.YWorldLimits(2) - tolerance;
    


    % Get observations and calculate reward
    non_normalized_observations = GetObservations();
    NextObs = NormalizeObservations(non_normalized_observations);
    if robot.sim_params.visualization_tools == true
        r = rateControl(1/robot.sim_params.sim_T);
        robot.viz(robot.sim_params.pose_array(:,robot.sim_params.iteration),robot.sim_params.path,robot.lidar_points_angles(:,1))
        drawnow
        waitfor(r);  % This ensures proper timing for visualization updates
    end
    [Reward, IsDone] = CalculateReward(NextObs,non_normalized_observations);
    % Update iteration
    robot.sim_params.iteration = robot.sim_params.iteration + 1;
    
    % Penalty for leaving the map boundaries (should never happen, anyway)
    if robot.pose(1) < x_min || robot.pose(1) > x_max || robot.pose(2) < y_min || robot.pose(2) > y_max
        disp("Robot attempted to enter the restricted boundary! Resetting simulation...");
        Reward = Reward-1000000; % Large penalty for approaching the map boundaries
        IsDone = true;
        return;
    end
end

% Calculate reward
function [reward, isDone] = CalculateReward(observations,non_normalized_observations)
    global robot
    isDone = false;
    reward = 0;

    % 1) Distance to goal reward
    goal_reward = 0;
    if non_normalized_observations(1) < 0.3 % TO DO observations are scaled between 0 and 1, find a value that works
        if robot.sim_params.visualization_tools == true
            disp("Goal reached!");
        end
        goal_reward = 50; % Large positive reinforcement
        isDone = true;
        robot.sim_params.success_counter = robot.sim_params.success_counter + 1;
        % get_new_goal();
    end
    reward = reward + goal_reward;  % Moderate positive reinforcement
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.goal_reached = [robot.sim_params.rewards.goal_reached ; goal_reward];
    end
    
    % 3) Orientation towards marker reward
    orientation_reward = reward + 5 * cos(observations(3) * pi);  % Cosine reward from direction error in [-1, 1] (+5 ~ -5)
    reward = reward + orientation_reward;
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.orientation = [robot.sim_params.rewards.orientation ; orientation_reward];
    end
    
    % 2) Distance to marker reward
    marker_reward = 6 * (0.5 + cos(observations(2) * pi)); % [-1, 1] (+9 ~ -3)
    if non_normalized_observations(2) < 1.2 && robot.sim_params.path_index < height(robot.sim_params.path) % aboid considering the last marker a simpl emarker
        if robot.sim_params.visualization_tools == true
            disp("Marker reached!");
        end
        marker_reward = marker_reward + 10;
        robot.sim_params.path_index = min(robot.sim_params.path_index + 1, height(robot.sim_params.path));
    end
    reward = reward + marker_reward;  % Moderate positive reinforcement
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.marker_reached = [robot.sim_params.rewards.marker_reached ; marker_reward];
    end

    % 4) Collision penalty
    obstacle_penalty = 0;
    collision_penalty = 0;
    safety_threshold = 0.4;
    for i = 6:15
        % Penalty for approaching obstacles
        obstacle_penalty = obstacle_penalty -10 * exp(-16 * observations(i));
        if non_normalized_observations(i) < safety_threshold
            isDone = true;  % Still terminate on real collision
            % Penalty for collision
            collision_penalty = collision_penalty - 240 * (0.1 + exp(-6 * robot.sim_params.iteration / 600)); % Exponential penalty for collisions
            robot.sim_params.success_counter = 0;
            if robot.sim_params.visualization_tools == true
                disp("Collision!");
            end
        end
    end
    reward = reward + obstacle_penalty;
    reward = reward + collision_penalty;
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.obstacle = [robot.sim_params.rewards.obstacle ; obstacle_penalty];
        robot.sim_params.rewards.collision = [robot.sim_params.rewards.collision ; collision_penalty];
    end

    % 5) Local minima penalty
    local_minima_penalty = 0;
    if observations(4) == 0.0 && observations(5) == 0.0
        if robot.sim_params.visualization_tools == true
            disp("Local minima!");
        end
        local_minima_penalty = - 16;  % Large negative reinforcement
    end
    reward = reward + local_minima_penalty;
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.local_minima = [robot.sim_params.rewards.local_minima ; local_minima_penalty];
    end
    
    % 6) Time limit penalty
    time_limit_penalty = 2 * (1 - exp(2 * robot.sim_params.iteration / 600));
    if robot.sim_params.iteration > 600.0
        if robot.sim_params.visualization_tools == true
            disp("Time limit reached!");
        end
        isDone = true;
    end
    reward = reward + time_limit_penalty;  % Match plot function
    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.time_limit = [robot.sim_params.rewards.time_limit ; time_limit_penalty];
    end


end
%% 
% 

% Reset Simulation
function [InitialObservation, Info] = ResetFunction()
    global robot
    
    if robot.sim_params.visualization_tools == true && robot.sim_params.iteration > 1 % && 0
        % Plot rewards over iterations
        figure(3); % Open a new figure window
        clf; % Clear the figure

        % First subplot: Rewards
        subplot(2, 1, 1); % Create the first subplot
        hold on;
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.goal_reached, 'g', 'DisplayName', 'Goal Reached');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.marker_reached, 'b', 'DisplayName', 'Marker Reached');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.orientation, 'm', 'DisplayName', 'Orientation');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.obstacle, 'r', 'DisplayName', 'Obstacle');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.collision, 'k', 'DisplayName', 'Collision');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.local_minima, 'c', 'DisplayName', 'Local Minima');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.rewards.time_limit, 'y', 'DisplayName', 'Time Limit');
        hold off;
        xlabel('Iteration');
        ylabel('Reward');
        title('Rewards Over Iterations');
        legend('show');
        ylim([-15 +15]); % Set y-axis scale to 0 to 1
        grid on;

        % Second subplot: DWA parameters
        subplot(2, 1, 2); % Create the second subplot
        hold on;
        plot(1:robot.sim_params.iteration-1, robot.sim_params.actions.alpha, 'r', 'DisplayName', 'Alpha');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.actions.beta, 'g', 'DisplayName', 'Beta');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.actions.gamma, 'b', 'DisplayName', 'Gamma');
        plot(1:robot.sim_params.iteration-1, robot.sim_params.actions.delta, 'm', 'DisplayName', 'Delta');
        hold off;
        xlabel('Iteration');
        ylabel('DWA Parameters');
        title('DWA Parameters Over Iterations');
        legend('show');
        ylim([0 1]); % Set y-axis scale to 0 to 1
        grid on;

        drawnow;
    end

    if robot.sim_params.visualization_tools == true
        % Reset visualizer
        robot.viz.reset();
        robot.sim_params.rewards.goal_reached = [];
        robot.sim_params.rewards.marker_reached = [];
        robot.sim_params.rewards.orientation = [];
        robot.sim_params.rewards.obstacle = [];
        robot.sim_params.rewards.collision = [];
        robot.sim_params.rewards.local_minima = [];
        robot.sim_params.rewards.time_limit = [];

        robot.sim_params.actions.alpha = [];
        robot.sim_params.actions.beta = [];
        robot.sim_params.actions.gamma = [];
        robot.sim_params.actions.delta = [];
    end


    Info = zeros(15, 1);

    % Reset robot path index
    robot.sim_params.path_index = 1;

    % Erase robot pose array
    robot.sim_params.pose_array = zeros(3, numel(0:robot.sim_params.sim_T:robot.sim_params.sim_time_limit));

    % Reset robot pose
    robot.pose = [0, 0, 0];
    robot.previous_pose = [0, 0, 0];

    % Reset robot velocity
    robot.velocity = [0, 0];

    % Reset time
    robot.sim_params.iteration = 1;

    % Get observations
    non_normalized_observations = GetObservations();
    InitialObservation = NormalizeObservations(non_normalized_observations);
    if robot.sim_params.visualization_tools == true
        disp("Reset simulation!");
    end

    robot.sim_params.episode = robot.sim_params.episode + 1;
    
    if mod(robot.sim_params.episode, round(robot.sim_params.episode_save_interval / 10)) == 0
        disp("New goal 100: " + int2str(robot.sim_params.episode));
        get_new_goal();
        robot.sim_params.success_counter = 0;
    elseif robot.sim_params.success_counter >= robot.sim_params.success_counter_threshold
        disp("New goal success: " + int2str(robot.sim_params.episode));
        get_new_goal();
        robot.sim_params.success_counter = 0;
    end

    % if mod(robot.sim_params.episode, robot.sim_params.episode_save_interval) == 0
    %     global agentObj;
    %     save_name = "trainedAgent_" + int2str(robot.sim_params.episode) + ".mat";
    %     save_location = fullfile(pwd, "trained_agents", save_name);
    %     save(save_location, 'agentObj');
    %     disp("Agent saved!");
    % end

end
%% 
% 

function Observations = GetObservations()
    % Observations:
    % Define observation space (continuous)

    % 1)  Distance Goal
    % 2)  Distance to next path marker
    % 3)  Relative Direction to next path marker

    % 4) Linear velocity
    % 5) Angular velocity

    % 6) Closest distance sector 1 (1-12)
    % 7) Closest distance sector 2 (13-24)
    % 8) Closest distance sector 3 (25-36)
    % 9) Closest distance sector 4 (37-48)
    % 10) Closest distance sector 5 (49-60)
    % 11) Closest distance sector 6 (61-72)
    % 12) Closest distance sector 7 (73-84)
    % 13) Closest distance sector 8 (85-96)
    % 14) Closest distance sector 9 (97-108)
    % 15) Closest distance sector 10 (109-120)

    global robot

    Observations = zeros(15, 1); % Pre-allocate for 8 observation values

    % Read lidar data
    robot.lidar_points_angles = [robot.lidar(robot.pose'), robot.lidar.scanAngles'];

    % 1)  Distance Goal
    Observations(1) = sqrt((robot.sim_params.goal(1)-robot.pose(1))^2 + (robot.sim_params.goal(2)-robot.pose(2))^2);

    % 2)  Distance to next path marker
    % 3)  Relative Direction to next path marker
    current_objective = robot.sim_params.path(robot.sim_params.path_index,:);
    Observations(2) = sqrt((current_objective(1)-robot.pose(1))^2 + (current_objective(2)-robot.pose(2))^2);
    Observations(3) = mod(atan2(current_objective(2)-robot.pose(2), current_objective(1)-robot.pose(1)) - robot.pose(3) + pi, 2*pi) - pi;
    
    % 4) Linear velocity
    % 5) Angular velocity
    Observations(4) = robot.velocity(1);
    Observations(5) = robot.velocity(2);

    % 6) Closest distance sector 1 (1-12)
    % 7) Closest distance sector 2 (13-24)
    % 8) Closest distance sector 3 (25-36)
    % 9) Closest distance sector 4 (37-48)
    % 10) Closest distance sector 5 (49-60)
    % 11) Closest distance sector 6 (61-72)
    % 12) Closest distance sector 7 (73-84)
    % 13) Closest distance sector 8 (85-96)
    % 14) Closest distance sector 9 (97-108)
    % 15) Closest distance sector 10 (109-120)
    closest_points = closer_lidar_points(robot.lidar_points_angles, 20.0);
    for i = 6:15
        Observations(i) = closest_points(i-5,1);
    end
    
end

function Observations = NormalizeObservations(Observations)
    % Observations:
    % Define observation space (continuous)

    % 1)  Distance Goal
    % 3)  Distance to next path marker
    % 4)  Relative Direction to next path marker

    % 5) Linear velocity
    % 6) Angular velocity

    % 7) Closest distance sector 1 (1-12)
    % 8) Closest distance sector 2 (13-24)
    % 9) Closest distance sector 3 (25-36)
    % 10) Closest distance sector 4 (37-48)
    % 11) Closest distance sector 5 (49-60)
    % 12) Closest distance sector 6 (61-72)
    % 13) Closest distance sector 7 (73-84)
    % 14) Closest distance sector 8 (85-96)
    % 15) Closest distance sector 9 (97-108)
    % 16) Closest distance sector 10 (109-120)

    global robot

    % Normalize distance to goal (1) - Scale to [0,1] using distance from origin to goal
    goal_distance_from_origin = sqrt(robot.sim_params.goal(1)^2 + robot.sim_params.goal(2)^2);
    Observations(1) = min(Observations(1) / goal_distance_from_origin, 1.0);
    
    % Normalize distance to next path marker (3) - Scale to [0,1]
    Observations(2) = min(Observations(2) / robot.sim_params.marker_max_separation, 1.0);
    
    % Relative direction to next path marker (4) is already normalized between -pi and pi
    % Normalize to [-1,1]
    Observations(3) = Observations(3) / pi;

    % Normalize velocities (11,12)
    Observations(4) = min( Observations(4) / robot.v_max, 1.0);  % Linear velocity normalized by max velocity
    Observations(5) = max(min(Observations(5) / robot.omega_max, 1.0), -1.0);  % Angular velocity normalized by max angular velocity
    
    % Normalize lidar distances (7-16) - Scale to [0,1] based on max lidar range
    for i = 6:15
        Observations(i) = min(Observations(i) / (robot.lidar.maxRange/6.0), 1.0);
    end
end


% Filter lidar
function filtered_lidar = closer_lidar_points(lidar_data, ~) 
    % Split lidar data into 10 sectors and find closest point in each
    
    % Get the number of lidar points
    num_points = size(lidar_data, 1);
    
    % Calculate the number of points per sector (assuming evenly distributed angles)
    points_per_sector = ceil(num_points / 10);
    
    % Initialize array to store closest points from each sector
    closest_points = zeros(10, 2);
    
    % Process each sector
    for sector = 1:10
        % Calculate start and end indices for this sector
        start_idx = (sector-1) * points_per_sector + 1;
        end_idx = min(sector * points_per_sector, num_points);
        
        % Extract data for this sector
        sector_data = lidar_data(start_idx:end_idx, :);
        
        % Find the closest point in this sector
        if ~isempty(sector_data)
            [min_range, min_idx] = min(sector_data(:, 2));
            closest_points(sector, :) = sector_data(min_idx, :);
        else
            % If sector is empty (shouldn't happen with evenly distributed data)
            closest_points(sector, :) = [0, 30]; % Default to max range
        end
    end
    
    % Return the filtered lidar data
    filtered_lidar = closest_points;
end
%% 
% 

% DWA Calculation
function [v_opt, omega_opt] = CalculateDWA(robot)

    v_max = min([robot.v_max, robot.v_max + robot.a_max * robot.DWA.T]);
    v_min = max([0, -robot.a_max * robot.DWA.T]);
    omega_max = min([+robot.omega_max, +robot.omega_max + robot.alpha_max * robot.DWA.T]);
    omega_min = max([-robot.omega_max, -robot.omega_max - robot.alpha_max * robot.DWA.T]);

    deltaV = (v_max - v_min) / robot.DWA.Ndata;
    deltaW = (omega_max - omega_min) / robot.DWA.Ndata;
    v_samples = v_min:deltaV:v_max;
    omega_samples = omega_min:deltaW:omega_max;

    best_score = -Inf;
    v_opt = 0;
    omega_opt = 0;

    % Preallocate matrices for scores
    heading_scores = zeros(length(v_samples), length(omega_samples));
    obstacle_scores = zeros(length(v_samples), length(omega_samples));
    velocity_scores = zeros(length(v_samples), length(omega_samples));
    energy_scores = zeros(length(v_samples), length(omega_samples));

    for i = 1:length(v_samples)
        for j = 1:length(omega_samples)
            v = v_samples(i);
            omega = omega_samples(j);

            if omega == 0
                theta_sim = robot.pose(3);
                x_sim = robot.pose(1) + v * cos(robot.pose(3)) * robot.DWA.T;
                y_sim = robot.pose(2) + v * sin(robot.pose(3)) * robot.DWA.T;
            else
                radius = v / omega;
                theta_sim = robot.pose(3) + omega * robot.DWA.T;

                cx = robot.pose(1) - radius * sin(robot.pose(3)); % rotation center x
                cy = robot.pose(2) + radius * cos(robot.pose(3)); % rotation center y

                x_sim = cx + radius * sin(theta_sim);
                y_sim = cy - radius * cos(theta_sim);
            end

            % Calculate scores
            % Heading score
            heading_scores(i, j) = CalculateHeadingScore(x_sim, y_sim, theta_sim, robot);
            % Obstacle score
            if i ~= 1 % If we detect a collision, we won't calculate the obstacle score for higher velocities
                if obstacle_scores(i-1, j) == 0
                    obstacle_scores(i, j) = 0;
                else
                    obstacle_scores(i, j) = CalculateObstacleScore(x_sim, y_sim, robot);
                end
            else
                obstacle_scores(i, j) = CalculateObstacleScore(x_sim, y_sim, robot);
            end
            % Velocity score
            velocity_scores(i, j) = v / robot.v_max;
            % Energy score
            energy_scores(i, j) = 1 - min((robot.mass * v^2 + robot.inertia * omega^2) / (2 * robot.mass * robot.v_max^2), 1);

            % Calculate total score
            score = robot.DWA.alpha * heading_scores(i, j) + ...
                    robot.DWA.beta * obstacle_scores(i, j) + ...
                    robot.DWA.gamma * velocity_scores(i, j) + ...
                    robot.DWA.delta * energy_scores(i, j);

            if score > best_score
                best_score = score;
                v_opt = v;
                omega_opt = omega;
            end
        end
    end

    if robot.sim_params.visualization_tools == true && 0
        % Plot the scores
        figure(2);

        % Plot Heading Score
        subplot(2, 2, 1);
        hold on;
        for i = 1:length(v_samples)
            for j = 1:length(omega_samples)
                score = heading_scores(i, j);
                color = [1 - score, score, 0]; % Red to green gradient based on score
                text(v_samples(i), omega_samples(j), sprintf('%.2f', score), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', color);
            end
        end
        hold off;
        title('Heading Score');
        xlabel('Velocity (v)');
        ylabel('Angular Velocity (\omega)');
        set(gca, 'Color', 'w'); % Set white background
        grid on;

        % Plot Obstacle Score
        subplot(2, 2, 2);
        hold on;
        for i = 1:length(v_samples)
            for j = 1:length(omega_samples)
                score = obstacle_scores(i, j);
                color = [1 - score, score, 0]; % Red to green gradient based on score
                text(v_samples(i), omega_samples(j), sprintf('%.2f', score), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', color);
            end
        end
        hold off;
        title('Obstacle Score');
        xlabel('Velocity (v)');
        ylabel('Angular Velocity (\omega)');
        set(gca, 'Color', 'w'); % Set white background
        grid on;

        % Plot Velocity Score
        subplot(2, 2, 3);
        hold on;
        for i = 1:length(v_samples)
            for j = 1:length(omega_samples)
                score = velocity_scores(i, j);
                color = [1 - score, score, 0]; % Red to green gradient based on score
                text(v_samples(i), omega_samples(j), sprintf('%.2f', score), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', color);
            end
        end
        hold off;
        title('Velocity Score');
        xlabel('Velocity (v)');
        ylabel('Angular Velocity (\omega)');
        set(gca, 'Color', 'w'); % Set white background
        grid on;

        % Plot Energy Score
        subplot(2, 2, 4);
        hold on;
        for i = 1:length(v_samples)
            for j = 1:length(omega_samples)
                score = energy_scores(i, j);
                color = [1 - score, score, 0]; % Red to green gradient based on score
                text(v_samples(i), omega_samples(j), sprintf('%.2f', score), ...
                    'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', color);
            end
        end
        hold off;
        title('Energy Score');
        xlabel('Velocity (v)');
        ylabel('Angular Velocity (\omega)');
        set(gca, 'Color', 'w'); % Set white background
        grid on;
    end

end

function score = CalculateHeadingScore(x_sim, y_sim, theta_sim, robot)
    current_objective = robot.sim_params.path(robot.sim_params.path_index,:);
    goal_heading = atan2(current_objective(2) - y_sim, current_objective(1) - x_sim);
    heading_diff = abs(angdiff(theta_sim, goal_heading));
    score = (pi - abs(heading_diff)) / pi;
end

function score = CalculateObstacleScore(x_sim, y_sim, robot)
    % Extract lidar points in polar coordinates (range and angle)
    ranges = robot.lidar_points_angles(:, 1);
    angles = robot.lidar_points_angles(:, 2);

    % Convert lidar points to Cartesian coordinates in the robot's frame
    lidar_x_robot = ranges .* cos(angles);
    lidar_y_robot = ranges .* sin(angles);

    % Transform lidar points to world coordinates
    lidar_x_world = lidar_x_robot * cos(robot.pose(3)) - lidar_y_robot * sin(robot.pose(3)) + robot.pose(1);
    lidar_y_world = lidar_x_robot * sin(robot.pose(3)) + lidar_y_robot * cos(robot.pose(3)) + robot.pose(2);

    % Compute distances from all lidar points to (x_sim, y_sim)
    distances = sqrt((lidar_x_world - x_sim).^2 + (lidar_y_world - y_sim).^2);

    % Find the minimum distance to the closest obstacle
    min_distance = min(distances);

    % Cap the distance to aboid negative scores
    min_distance = max(min_distance, 0.4); 

    % Normalize the score based on the minimum distance
    score = min((min_distance - 0.4) / (2 - 0.4), 1.0);
end

%%
%

function randomPoint = get_new_goal()
    global robot

    % Define the vertices of the polygon
    vertices = [
        28.4, 1.3;
        28.4, -1.1;
        -19.7, -1.6;
        -20.0, 14.0;
        -17.8, 14.0;
        -17.9, 0.8
    ];

    hardcoded_obstacles = [
        22.7 , -0.6 ; % yellow cylinder
        19.2 , -1.0 ; % box
        17.2 , 1.2  ; % mail-box
        14.0 , 1.0  ; % shelf
        8.0  , 0.8  ; % green cylinder
        -5.8 , 0.3  ; % gray cylinder
        -11.9, 0.6  ; % pink cylinder
        -19.3, -1.2 ; % red cylinder
        -18.5, 4.8  ; % cyan cylinder
        -19.5, 8.2  ; % table
    ];

    % Get bounding box of the polygon
    minX = min(vertices(:, 1));
    maxX = max(vertices(:, 1));
    minY = min(vertices(:, 2));
    maxY = max(vertices(:, 2));

    while true
        % Generate a random point within the bounding box
        x = minX + (maxX - minX) * rand();
        y = minY + (maxY - minY) * rand();

        % Check if the point is at least 2.0 units away from all hardcoded obstacles
        distances = sqrt((hardcoded_obstacles(:,1) - x).^2 + (hardcoded_obstacles(:,2) - y).^2);
        if any(distances < 2.0)
            continue; % Skip this iteration and generate a new point
        end

        % Check if the point is inside the polygon
        if inpolygon(x, y, vertices(:, 1), vertices(:, 2)) && sqrt(x^2 + y^2) > 4.0
            randomPoint = [x, y];

            robot.sim_params.goal = randomPoint;

            if generateObstaclesRoutine(vertices)
                break;
            else
                continue; % Retry if obstacle generation fails
            end
        end
    end
end


function path = plan_path(start_pose, goal_pose, map)
    FreeThreshold = map.FreeThreshold;
    OccupiedThreshold = map.OccupiedThreshold;
    GridLocationInWorld = map.GridLocationInWorld;

    % Convert map to a matrix
    mapMatrix = getOccupancy(map);
    mapMatrix = flipud(mapMatrix);
    map = occupancyMap(mapMatrix, map.Resolution);
    
    % Restore thresholds and grid location
    map.FreeThreshold = FreeThreshold;
    map.OccupiedThreshold = OccupiedThreshold;
    map.GridLocationInWorld = GridLocationInWorld;

    % Inflate obstacles to create a safety margin
    inflationRadius = 0.5; % Adjust this value as needed
    inflate(map, inflationRadius);

    % Convert start and goal poses to grid indices
    [xGridStart, yGridStart] = world2grid(map, start_pose);
    start_idx = [yGridStart, xGridStart];
    [xGridGoal, yGridGoal] = world2grid(map, goal_pose);
    goal_idx = [yGridGoal, xGridGoal];

    % Use A* planner to compute the path
    planner = plannerAStarGrid(map);
    path_idx = plan(planner, start_idx, goal_idx);
    path_idx = [path_idx(:,2), path_idx(:,1)];

    % Convert the path back to world coordinates
    path = grid2world(map, path_idx);
    path = downsamplePath(path, 20);
end


function [xGrid, yGrid] = world2grid(map, pose)
    % Convert world coordinates to grid indices
    res = map.Resolution; % cells per meter
    origin = map.GridLocationInWorld;
    idx = round((pose - origin) * res) + 1;
    
    % Split the indices into x and y components
    xGrid = idx(1);
    yGrid = idx(2);

end

function pose = grid2world(map, idx)
    % Convert grid indices back to world coordinates
    res = map.Resolution;
    origin = map.GridLocationInWorld;
    pose = (idx - 1) / res + origin;
end

function reduced_path = downsamplePath(path, step)
    if nargin < 2
        step = 20; % Default step size
    end
    
    % Select every `step` points, including the first and last
    indices = unique([1:step:size(path, 1), size(path, 1)]);
    reduced_path = path(indices, :);
end

function success = generateObstaclesRoutine(vertices)
    global robot
    % Generate random obstacles and update the map
    maxAttempts = 4; % Maximum attempts to generate a valid map
    for attempt = 1:maxAttempts
        % Generate random obstacles
        map_copy = generateRandomObstacles(robot.map,vertices);
        
        closedVertices = [vertices; vertices(1, :)];
        % Visualize the map for debugging
        if robot.sim_params.visualization_tools == true
            figure(1); % Open a figure window
            show(map_copy); % Display the occupancy map
            hold on;
            plot(closedVertices(:, 1), closedVertices(:, 2), 'r-', 'LineWidth', 2); % Plot the polygon boundary
            scatter(robot.sim_params.goal(1), robot.sim_params.goal(2), 100, 'g', 'filled'); % Plot the goal
            hold off;
            drawnow; % Update the figure
        end
        % Plan a path with the updated map
        try
            % Explicitly pass map_copy to plan_path
            temp_path = plan_path([0, 0], robot.sim_params.goal, map_copy);
    
            % If a valid path is found, update the robot's map and break
            if ~isempty(temp_path)
                robot.map_copy = map_copy; % Use the updated map
                break;
            end
        catch
            % If planning fails, retry with new obstacles
            if robot.sim_params.visualization_tools == true
                disp("Path planning failed. Retrying with new obstacles...");
            end
            success = false;
            return;
        end
    end
    
    % If no valid path is found after maxAttempts, throw an error
    % This if should never be reached due to the previous return statement
    % but it's a safeguard in case the loop doesn't break
    if isempty(temp_path)
        disp("Unable to find a valid path after maxAttempts attempts.");
        success = false;
        return;
    end
    
    % Plan the actual path that we will feed to the DWA
    robot.sim_params.path = plan_path([0, 0], robot.sim_params.goal, robot.map);

    % Save the updated map to a file and register it in the base workspace
    assignin('base', 'map_copy', robot.map_copy); % Register the map in the base workspace
    
    % Update the visualizer with the new map
    release(robot.viz); % Release the visualizer to allow property changes
    robot.viz.mapName = 'map_copy'; % Pass the map name to the visualizer
    attachLidarSensor(robot.viz, robot.lidar); % Reattach the lidar sensor
    
    success = true;
    return; % Exit the function successfully
end

function updatedMap = generateRandomObstacles(originalMap, vertices)
    % Copy the original map
    updatedMap = copy(originalMap);

    % Parameters for obstacle generation
    numObstacles = randi([6, 8]); % Random number of obstacles
    minRadius = 0.4; % Minimum radius of obstacles
    maxRadius = 0.8; % Maximum radius of obstacles

    minX = min(vertices(:, 1));
    maxX = max(vertices(:, 1));
    minY = min(vertices(:, 2));
    maxY = max(vertices(:, 2));

    % Generate random obstacles
    for i = 1:numObstacles
        while true
            % Random center within map boundaries
            xCenter = minX + (maxX - minX) * rand();
            yCenter = minY + (maxY - minY) * rand();

            % Check if the center is inside the polygon
            if inpolygon(xCenter, yCenter, vertices(:, 1), vertices(:, 2)) && sqrt(xCenter^2 + yCenter^2) > 4.0
                % Random radius
                radius = minRadius + (maxRadius - minRadius) * rand();

                % Add the circular obstacle to the map
                addCircularObstacle(updatedMap, [xCenter, yCenter], radius);
                break; % Exit the loop once a valid obstacle is added
            end
        end
    end
end

function addCircularObstacle(map, center, radius)
    % Get map resolution
    resolution = map.Resolution;

    % Convert center and radius to grid indices
    [xGrid, yGrid] = world2grid(map, center);
    yGrid = map.GridSize(1) - yGrid; % Swich the y axis
    radiusGrid = round(radius * resolution);

    % Create a circular mask
    [x, y] = meshgrid(-radiusGrid:radiusGrid, -radiusGrid:radiusGrid);
    circleMask = (x.^2 + y.^2) <= radiusGrid^2;

    % Apply the mask to the map
    xRange = (xGrid - radiusGrid):(xGrid + radiusGrid);
    yRange = (yGrid - radiusGrid):(yGrid + radiusGrid);

    % Ensure indices are within map bounds
    validX = xRange >= 1 & xRange <= map.GridSize(2);
    validY = yRange >= 1 & yRange <= map.GridSize(1);

    % Adjust the ranges and mask to match the valid region
    xRange = xRange(validX);
    yRange = yRange(validY);
    circleMask = circleMask(validY, validX);

    % Check if the mask and ranges are valid
    if isempty(xRange) || isempty(yRange) || isempty(circleMask)
        warning('Obstacle could not be added: out of map bounds or invalid mask.');
        return;
    end

    % Manually paint the pixels in the occupancy grid
    mapMatrix = getOccupancy(map); % Get the occupancy grid as a matrix
    for i = 1:length(yRange)
        for j = 1:length(xRange)
            if circleMask(i, j)
                mapMatrix(yRange(i), xRange(j)) = 1; % Mark as occupied
            end
        end
    end

    % Set the updated occupancy values back to the map
    setOccupancy(map, mapMatrix);

end

function SetupFcn()
    % Robot parameters
    global robot
    robot.mass = 1.0;
    robot.inertia = 1.0;
    robot.v_max = 1.0;
    robot.omega_max = 1.2;
    robot.a_max = 1.0;
    robot.alpha_max = 3.0;
    robot.radius = 0.25;
    robot.pose = [0, 0, 0]; % [x, y, theta]
    robot.previous_pose = [0, 0, 0]; % [x, y, theta]
    robot.velocity = [0, 0]; % [v, omega]

    robot.sim_params.iteration = 1;
    robot.sim_params.sim_T = 0.1;    % Simulation period
    robot.sim_params.sim_time_limit = 60.0;
    robot.sim_params.pose_array = zeros(3,numel(0:robot.sim_params.sim_T:robot.sim_params.sim_time_limit)); % Pre alocate the pose array with the same size of the simulation time
    robot.sim_params.goal = [0, 0];
    robot.sim_params.path = [];
    robot.sim_params.path_index = 1;
    robot.sim_params.marker_max_separation = 3.0;
    robot.sim_params.episode = 1;
    robot.sim_params.visualization_tools = false;
    robot.sim_params.episode_save_interval = 1000; % Save the agent every 1000 episodes
    robot.sim_params.success_counter = 0;
    robot.sim_params.success_counter_threshold = 20; % Number of successful episodes before reset
    robot.sim_params.use_parallel = true; % Use parallel training (true/false)

    if robot.sim_params.visualization_tools == true
        robot.sim_params.rewards.goal_reached = [];
        robot.sim_params.rewards.marker_reached = [];
        robot.sim_params.rewards.orientation = [];
        robot.sim_params.rewards.obstacle = [];
        robot.sim_params.rewards.collision = [];
        robot.sim_params.rewards.local_minima = [];
        robot.sim_params.rewards.time_limit = [];

        robot.sim_params.actions.alpha = [];
        robot.sim_params.actions.beta = [];
        robot.sim_params.actions.gamma = [];
        robot.sim_params.actions.delta = [];
    end

    % Load map
    load corridor_map.mat
    robot.map = map;
    robot.map_copy = map;

    %%
    %

    % Lidar
    robot.lidar = LidarSensor;
    robot.lidar.sensorOffset = [0.16,0];
    robot.lidar.scanAngles = linspace(-2.35619, 2.35619, 120);  % sample_size=120 from URDF
    robot.lidar.maxRange = 30.0;
    %%
    %

    % Create visualizer
    robot.viz = Visualizer2D;
    robot.viz.hasWaypoints = true;
    robot.viz.mapName = 'map_copy';
    attachLidarSensor(robot.viz,robot.lidar);
    %% 
    % 

    % DWA Parameters
    robot.DWA.alpha = 0.55;   % Heading coefficient
    robot.DWA.beta = 0.70;    % Obstacle distance coefficient
    robot.DWA.gamma = 0.15;   % Velocity coefficient
    robot.DWA.delta = 0.0;    % Energy coefficient
    robot.DWA.T = 1.0;        % Time horizon
    robot.DWA.Ndata = 8;      % Amount of velocity intervals

    %%
    % Reset simulation before everything
    get_new_goal();
    ResetFunction();
end

