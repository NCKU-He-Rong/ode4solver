function dx = DCMotorControlSystem(t, x)
    
    % Defeine the DC Motor paramter
    a = 8.464512101120487;
    b = 32.64864577071974;
    c_pos = 27.72313746485287;
    c_neg = 26.477893489181326;

    alpha = (a / b);
    beta = (1.0 / b);
    gamma1 = (c_pos / b);
    gamma2 = (c_neg / b);

    % Define the gains
    kp = 150;
    ki = 60;
    kd = 20;

    % Calculate the desire 
    desire = (pi / 4.0) * (1 + sin(2 * pi * 1 * t - pi / 2.0));
    desire_d = (pi / 4.0) * (2 * pi * 1) * cos(2 * pi * 1 * t - pi / 2.0);
    desire_dd = - (pi / 4.0) * (2 * pi * 1) * (2 * pi * 1) * sin(2 * pi * 1 * t - pi / 2.0);

    % Calculate the error
    error = desire - x(1);
    error_dot = desire_d - x(2);
    I = x(3);

    % Calculate the input
    u_pid = beta * (kp * error + kd * error_dot + ki * I);
    u_ff = alpha * x(2) + beta * desire_dd + gamma1 * max(0, sign(x(2))) + gamma2 * min(0, sign(x(2)));
    u = u_pid + u_ff;

    
    % Initial the dx
    dx = zeros(2, 1);
    
    % Define the dx / dt
    dx(1) = x(2);
    dx(2) = - a * x(2) ...
           + b *  u ...
           - c_pos * max(0, sign(x(2))) ...
           - c_neg * min(0, sign(x(2)));
    dx(3) = error;

end
