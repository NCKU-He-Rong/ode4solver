function dx = DCMotorSystem(t, x)

    % Defeine the DC Motor paramter
    a = 8.464512101120487;
    b = 32.64864577071974;
    c_pos = 27.72313746485287;
    c_neg = 26.477893489181326;
    
    % Initial the dx
    dx = zeros(2, 1);
    
    % Define the dx / dt
    dx(1) = x(2);
    dx(2) = - a * x(2) ...
           + b *  6 * sin(2*pi*1*t) ...
           - c_pos * max(0, sign(x(2))) ...
           - c_neg * min(0, sign(x(2)));

end
