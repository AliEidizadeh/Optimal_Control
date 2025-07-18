%% First Optimal Control Project
%{
Non-interpolated Dynamic Programming
Ali Eidizadeh       ali80ei@gmail.com
Amirhadi Keyvan     amirhadikeyvan@gmail.com
%}
%%
function optimal_control_dp_simplified()
    % Optimal Control using Dynamic Programming - Simplified Version
    % System dynamics: x(k+1) = x(k) + u(k)
    % Cost function: J = x(N)^2 + 2*sum(u(k)^2)
    % Constraints: 0 ≤ x(k) ≤ 1.5, -1 ≤ u(k) ≤ 1

    %% Problem Setup
    clc;  % Clear command window
    fprintf('=== Optimal Control using Dynamic Programming ===\n');

    % Ask user for quantization step sizes
    q_x = input('Enter state quantization step (e.g., 0.1): ');
    q_u = input('Enter input quantization step (e.g., 0.1): ');

    % Generate quantized state and input grids
    states = 0:q_x:1.5;
    inputs = -1:q_u:1;
    N = 10;  % Number of stages in the time horizon

    % Display generated state and input grids
    fprintf('\nState grid (%d points):\n', length(states));
    disp(states);
    fprintf('Control grid (%d points):\n', length(inputs));
    disp(inputs);

    %% Backward Pass with Linear Interpolation
    % Initialize cost-to-go function J and optimal control table U_opt
    J = cell(1, N+1);     % J{k} stores the cost-to-go at time k
    U_opt = cell(1, N);   % U_opt{k} stores the optimal control at time k
    J{N+1} = states.^2;   % Terminal cost: quadratic penalty on final state

    fprintf('\n=== Backward Pass Calculations ===\n');
    % Loop backwards from stage N to 1
    for k = N:-1:1
        J_current = inf(1, length(states));  % Initialize current cost-to-go
        U_current = nan(1, length(states));  % Initialize current optimal control

        % Loop through all discrete states
        for i = 1:length(states)
            min_cost = 1e3;  % Initialize with a large cost
            best_u = 0;      % Placeholder for best control input
            x = states(i);   % Current state

            % Loop through all discrete control inputs
            for j = 1:length(inputs)
                u = inputs(j);            % Try input u
                x_next = x + u;           % Compute next state

                % Skip if next state violates constraints
                if x_next < 0 || x_next > 1.5
                    continue;
                end

                % Compute future cost using linear interpolation
                if k == N
                    cost_next = x_next^2;  % Terminal cost
                else
                    cost_next = linear_interp(states, J{k+1}, x_next);
                end

                % Total cost = control cost + future cost
                total_cost = 2*u^2 + cost_next;

                % Update minimum cost and best control if better
                if total_cost < min_cost
                    min_cost = total_cost;
                    best_u = u;
                end
            end

            % Store minimum cost and optimal input for current state
            J_current(i) = min_cost;
            U_current(i) = best_u;
        end

        % Store current stage results
        J{k} = J_current;
        U_opt{k} = U_current;

        % Print current stage results
        fprintf('\nStage k = %d\n', k-1);
        fprintf('%-10s %-12s %-12s\n', 'State', 'Optimal Cost', 'Optimal Control');
        for i = 1:length(states)
            fprintf('%-10.2f %-12.4f %-12.4f\n', states(i), J{k}(i), U_opt{k}(i));
        end
    end

    %% Forward Pass Simulation
    while true
        % Ask for initial condition or quit
        x0 = input('\nEnter initial state (0-1.5) or "q" to quit: ', 's');
        if strcmpi(x0, 'q')
            break;
        end
        x0 = str2double(x0);  % Convert input string to number

        % Validate input
        if isnan(x0) || x0 < 0 || x0 > 1.5
            fprintf('Invalid initial state!\n');
            continue;
        end

        % Initialize trajectory and control sequence
        x_traj = zeros(1, N+1);
        u_seq = zeros(1, N);
        x_traj(1) = x0;

        % Simulate optimal control forward in time
        for k = 1:N
            % Find closest state index
            [~, idx] = min(abs(states - x_traj(k)));
            u = U_opt{k}(idx);  % Get optimal control for current state

            % Clamp control input within allowed bounds
            u = max(-1, min(1, u));

            % Update state with control and clamp within bounds
            x_next = x_traj(k) + u;
            x_next = max(0, min(1.5, x_next));

            % Store control and next state
            u_seq(k) = u;
            x_traj(k+1) = x_next;
        end

        % Calculate total cost of the simulated trajectory
        total_cost = x_traj(end)^2 + 2*sum(u_seq.^2);

        %% Display Results
        fprintf('\n=== Optimal Results ===\n');
        fprintf('Initial state: %.2f\n', x0);
        fprintf('Total cost: %.4f\n', total_cost);

        fprintf('\nOptimal Control Sequence:\n');
        disp(u_seq);

        fprintf('\nState Trajectory:\n');
        disp(x_traj);

        % Plot state trajectory and control input
        close all
        figure;
        subplot(2,1,1);
        plot(0:N, x_traj, 'o-', 'LineWidth', 2, 'MarkerFaceColor', 'b');
        title('Optimal State Trajectory');
        xlabel('Stage');
        ylabel('State');
        grid on;
        ylim([0 1.5]);

        subplot(2,1,2);
        stairs(0:N-1, u_seq,'o-', 'LineWidth', 2, 'MarkerFaceColor', 'g');
        title('Optimal Control Sequence');
        xlabel('Stage');
        ylabel('Control');
        grid on;
        ylim([-1 1]);
    end

    fprintf('\nProgram terminated.\n');
end

%---------------------------------------------------------
function yq = linear_interp(x_grid, y_grid, xq)
    % Linear interpolation with boundary checking
    % x_grid: known x points
    % y_grid: known function values at x_grid
    % xq: query point to interpolate

    if xq < x_grid(1) || xq > x_grid(end)
        yq = inf;  % Outside range → assign infinite cost
        return;
    end

    % Find the index of the nearest lower bound
    idx = find(x_grid <= xq, 1, 'last');

    % If xq is at the end of grid, return last value
    if idx == length(x_grid)
        yq = y_grid(end);
    else
        % Compute weighted average between two nearest points
        dx = x_grid(idx+1) - x_grid(idx);
        alpha = (xq - x_grid(idx))/dx;
        yq = (1-alpha)*y_grid(idx) + alpha*y_grid(idx+1);
    end
end
