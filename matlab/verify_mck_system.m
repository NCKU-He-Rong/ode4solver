%% 【ODE4】 - Varification with C++ Implementation
clc
clear
close all 

%% Manual Setting
% MCK parameters
m = 1.0; 
c = 0.5; 
k = 10.0;
force = 0.0;

% initial state
x0 = [1.0; 0.0];

%% Load the Result From the C++ Implementation
data = readtable("mck_simulation_result.csv", "VariableNamingRule", "preserve");

% read the time and state
time = data.("Time(Seconds)");
state0 = data.("State0");
state1 = data.("State1");

%% Simulation with MATLAB built-in ode45
% define the mck system's state space
sys = @(t,x) [x(2);(1/m) * (force - c*x(2) - k*x(1))];

% solve
[t, x] = ode45(sys, time, x0);

%% Plot the result 
figure("Position", [556.2,181.8,975.2,705.6]);
plot(time, state0, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation');
hold on;
plot(time, x(:, 1), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45');

% Axis labels
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('State Variable: Position', 'FontSize', 12, 'FontWeight', 'bold');

% Title
title('Comparison of C++ Implementation and MATLAB ODE45', ...
      'FontSize', 15, 'FontWeight', 'bold');

% Legend
legend('Location', 'best', 'FontSize', 15);

% Grid and box
grid on;
box on;

% Improve figure aesthetics
set(gca, 'FontSize', 15, 'LineWidth', 1.2);

%%




