%% 【ODE4】 - Varification with C++ Implementation
clc
clear
close all 

%% Manual Setting
% MCK parameters
sigma = 10.0;
rho = 28.0;
beta = 8.0 / 3.0;

% initial state
x0 = [1.0; 1.0; 1.0];

%% Time 與 Input 設定
t_input = 0:0.01:30;

%% Load the Result From the C++ Implementation
data = readtable("LorenzResult.csv", "VariableNamingRule", "preserve");

% read the time and state
time_cpp = data.("Time(Seconds)");
state0_cpp = data.("State0");
state1_cpp = data.("State1");
state2_cpp = data.("State2");

%% Simulation with MATLAB built-in ode45
% define the mck system's state space
sys = @(t,x) [sigma * (x(2) - x(1)); 
              x(1) * (rho - x(3)) - x(2);
              x(1)*x(2) - beta * x(3)];

% solve
[t_ode45, x_ode45] = ode45(sys, t_input, x0);
[t_rk4, x_rk4] = RK4(sys, [t_input(1), t_input(end)], x0, 0.01);


%% Plot the result 
figure("Position", [556.2,181.8,975.2,705.6]);
plot3(state0_cpp, state1_cpp, state2_cpp,'LineWidth', 1.8, 'DisplayName', 'C++ Implementation');
hold on;
plot3(x_ode45(:, 1), x_ode45(:, 2), x_ode45(:, 3), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45');
plot3(x_rk4(:, 1), x_rk4(:, 2), x_rk4(:, 3), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE4');

% Axis labels
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('State Variable: Position', 'FontSize', 12, 'FontWeight', 'bold');

% Title
title('Comparison of C++ Implementation and MATLAB ODE45', ...
      'FontSize', 15, 'FontWeight', 'bold');

% Legend
legend('Location', 'best', 'FontSize', 15);

% Grid and box
axis equal;
grid on;
box on;

% Improve figure aesthetics
set(gca, 'FontSize', 15, 'LineWidth', 1.2);

%% === 位置誤差 ===
position_x_error = x_rk4(:,1) - state0_cpp;
position_y_error = x_rk4(:,2) - state1_cpp;
position_z_error = x_rk4(:,3) - state2_cpp;
figure;
plot(time_cpp, position_x_error, 'LineWidth', 1.5, "DisplayName", "x");
grid on; box on;hold on
plot(time_cpp, position_y_error, 'LineWidth', 1.5, "DisplayName", "x");
plot(time_cpp, position_z_error, 'LineWidth', 1.5, "DisplayName", "x");
xlabel('Time (s)'); ylabel('Position Error');
title('Position Error: MATLAB RK4 - C++');

%%





