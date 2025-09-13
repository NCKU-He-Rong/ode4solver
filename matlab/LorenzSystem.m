function dx = LorenzSystem(t, x)

    % Defeine the system paramter
    sigma = 10.0;
    rho = 28.0;
    beta = 8.0 / 3.0;
    
    % Initial the dx
    dx = zeros(3, 1);
    
    % Define the dx / dt
    dx(1) = sigma * (x(2) - x(1));
    dx(2) = x(1) * (rho - x(3)) - x(2);
    dx(3) = x(1)*x(2) - beta * x(3);

end
