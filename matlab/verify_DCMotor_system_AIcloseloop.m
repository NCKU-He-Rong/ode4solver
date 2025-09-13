clc; 
clear; 
close all;

%% Time 與 Desire設定
t_input = 0:0.001:5;
desire = (pi / 4.0) * (1 + sin(2 * pi * 1 * t_input - pi / 2.0));

%% 初始條件
x0 = [0.0; 0.0; 0.0];

%% 載入 C++ 結果
data = readtable("RL_Result.csv", "VariableNamingRule", "preserve");
time_cpp = data.("Time(Seconds)");
state0_cpp = data.("State0"); 
state1_cpp = data.("State1"); 

%% 使用自定義 RK4與內建ODE45模擬
[t_rk4, x_rk4] = RK4(@DCMotorControlSystem, [t_input(1), t_input(end)], x0, 0.001);
[t_ode, x_ode45] = ode45(@DCMotorControlSystem, t_input(1):0.001:t_input(end), x0);

%% === Plot 1：位置比較 ===
figure("Position", [474,295,864,613]);
plot(time_cpp, state0_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(t_rk4, x_rk4(:,1), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB RK4');
plot(t_ode, x_ode45(:,1), 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45');
plot(t_input, desire, 'LineWidth', 1.8, 'DisplayName', 'Desire');
xlabel('Time (s)'); ylabel('Position');
title('Position Comparison of C++ Implementation, MATLAB ODE45 and MATLAB ODE4');
legend('Location','best'); grid on; box on;axis tight

%% === Plot 2：速度比較 ===
figure("Position", [474,295,864,613]);
plot(time_cpp, state1_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(t_rk4, x_rk4(:,2), '--', 'LineWidth', 1.8, 'DisplayName', 'MATLAB RK4');
plot(t_ode, x_ode45(:,2), 'LineWidth', 1.8, 'DisplayName', 'MATLAB ODE45');
xlabel('Time (s)'); ylabel('Velocity');
title('Velocity Comparison of C++ Implementation, MATLAB ODE45 and MATLAB ODE4');
legend('Location','best'); grid on; box on;axis tight

%% === Plot 3：位置誤差 ===
position_error = x_rk4(1:end-1,1) - state0_cpp;
figure("Position", [474,295,864,613]);
plot(time_cpp, position_error, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error');
title('Position Error: MATLAB RK4 - C++');
grid on; box on;axis tight

%% === Plot 4：速度誤差 ===
velocity_error = x_rk4(1:end-1,2) - state1_cpp;
figure("Position", [474,295,864,613]);
plot(time_cpp, velocity_error, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity Error');
title('Velocity Error: MATLAB RK4 - C++');
grid on; box on;axis tight

%%

