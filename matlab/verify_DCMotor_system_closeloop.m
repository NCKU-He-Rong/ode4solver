clc; clear; close all;

%% Input 設定
t_input = 0:0.001:5;

%% Desire 設定
desire = (pi / 4.0) * (1 + sin(2 * pi * 1 * t_input - pi / 2.0));
desire_d = (pi / 4.0) * (2 * pi * 1) * cos(2 * pi * 1 * t_input - pi / 2.0);
desire_dd = - (pi / 4.0) * (2 * pi * 1) * (2 * pi * 1) * sin(2 * pi * 1 * t_input - pi / 2.0);

%% 載入 C++ 結果
data = readtable("DCMotorResult_Control.csv", "VariableNamingRule", "preserve");
time_cpp = data.("Time(Seconds)");
state0_cpp = data.("State0");  % 位置
state1_cpp = data.("State1");  % 速度

%% === Plot 1：位置比較 ===
figure;
plot(time_cpp, state0_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(time_cpp, desire, '--', 'LineWidth', 1.8, 'DisplayName', 'Desire Trajectory');
xlabel('Time (s)'); ylabel('Position');
title('Control Result vs Desire Trajectory');
legend('Location','best'); grid on; box on;

%% === Plot 2：速度比較 ===
figure;
plot(time_cpp, state1_cpp, 'LineWidth', 1.8, 'DisplayName', 'C++ Implementation'); hold on;
plot(time_cpp, desire_d, '--', 'LineWidth', 1.8, 'DisplayName', 'Desire Velocity');
xlabel('Time (s)'); ylabel('Velocity');
title('Control Result vs Desire Velocity');
legend('Location','best'); grid on; box on;

%% === Plot 3：位置誤差 ===
position_error = desire' - state0_cpp;
figure;
plot(time_cpp, position_error, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error');
title('Position Error');
grid on; box on;

%% === Plot 4：速度誤差 ===
velocity_error = desire_d' - state1_cpp;
figure;
plot(time_cpp, velocity_error, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Velocity Error');
title('Velocity Error');
grid on; box on;

%%

