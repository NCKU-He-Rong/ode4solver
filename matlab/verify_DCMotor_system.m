clc; clear; close all;

%% DC motor parameters
a = 8.464512101120487;
b = 32.64864577071974;
c_pos = 27.72313746485287;
c_neg = 26.477893489181326;

%% Input 設定
t_input = 0:0.001:5;
u_input = 6 * sin(2*pi*1*t_input);

%% 初始條件
x0 = [0.0; 0.0];

%% 載入 C++ 結果
data = readtable("DCMotorResult.csv", "VariableNamingRule", "preserve");
time_cpp = data.("Time(Seconds)");
state0_cpp = data.("State0");  % 位置
state1_cpp = data.("State1");  % 速度

%% 定義系統
sys = @(t,x) [
    x(2);
    - a * x(2) ...
    + b *  6 * sin(2*pi*1*t) ...
    - c_pos * max(0, sign(x(2))) ...
    - c_neg * min(0, sign(x(2)))
];

%% 使用自定義 RK4 模擬
[t_rk4, x_rk4] = RK4(sys, [t_input(1), t_input(end)], x0, 0.001);
[t_ode, x_ode45] = ode45(sys, [t_input(1):0.001:t_input(end)], x0);


%% === Plot 1：位置比較 ===
figure;
plot(time_cpp, state0_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(t_rk4, x_rk4(:,1), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB RK4');
plot(t_ode, x_ode45(:,1), 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45'); hold on;
xlabel('Time (s)'); ylabel('Position');
title('Position Comparison: C++ vs MATLAB RK4');
legend('Location','best'); grid on; box on;

%% === Plot 2：速度比較 ===
figure;
plot(time_cpp, state1_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(t_rk4, x_rk4(:,2), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB RK4');
plot(t_ode, x_ode45(:,2), 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45'); hold on;
xlabel('Time (s)'); ylabel('Velocity');
title('Velocity Comparison: C++ vs MATLAB RK4');
legend('Location','best'); grid on; box on;

%% === Plot 3：位置誤差 ===
position_error = x_rk4(:,1) - state0_cpp;
figure;
plot(time_cpp, position_error, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error');
title('Position Error: MATLAB RK4 - C++');
grid on; box on;

%% === Plot 4：速度誤差 ===
velocity_error = x_rk4(:,2) - state1_cpp;
figure;
plot(time_cpp, velocity_error, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity Error');
title('Velocity Error: MATLAB RK4 - C++');
grid on; box on;
