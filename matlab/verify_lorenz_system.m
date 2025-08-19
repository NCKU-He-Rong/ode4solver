%% 【ODE4】 - Varification with C++ Implementation
clc
clear
close all 

%% Manual Setting
% MCK parameters
sigma = 10.0;
beta = 28.0;
rho = 8.0 / 3.0;

% initial state
x0 = [1.0; 1.0; 1.0];

%% Load the Result From the C++ Implementation
data = readtable("lorenz_simulation_result.csv", "VariableNamingRule", "preserve");

% read the time and state
time = data.("Time(Seconds)");
state0 = data.("State0");
state1 = data.("State1");
state2 = data.("State2");

%% Simulation with MATLAB built-in ode45
% define the mck system's state space
sys = @(t,x) [sigma * (x(2) - x(1)); x(1) * (rho - x(3)) - x(2);x(1)*x(2) - beta * x(3)];

% solve
[t, x] = ode45(sys, time, x0);

%% Plot the result 
figure("Position", [556.2,181.8,975.2,705.6]);
plot3(state0, state1, state2,'LineWidth', 1.8, 'DisplayName', 'C++ Implementation');
hold on;
plot3(x(:, 1), x(:, 2), x(:, 3), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45');

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




