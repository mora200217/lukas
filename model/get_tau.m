function [tau,b ] = get_tau(m, t, theta)
    % Linealización 
    b = theta - m * t; 
    % y = m x + b 
    % Despejamos 
    % -b / m = x (Intercepto y tau) 
    tau = -b / m; 

end