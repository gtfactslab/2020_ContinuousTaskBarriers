function [h, y] = ReachAB(X, alpha, t, t0, s, task_num)
    
    gamma = 10;    
    p = 6;
    sigma = [0.7, 0.2];
    theta_k = pi/2;
    k = theta_k/(2*sigma(1));
    c = 1;
    theta0 = sign(k)*pi/2;
    
    x_new = k*X(1);
    y_new = k*X(2) + 1;
    
    H = [1 0; 0 1];

    ca_x = 0.8; ca_y = 0.4; 
    cb_x = -0.2; cb_y = -0.6;
    cc_x = -1; cc_y = 0.2; 
    
    PA = [1/(0.3)^2 0; 0 1/(0.2)^2];    
    PB = [1/(0.35)^2 0; 0 1/(0.1)^2];
    PC = [1/(0.2)^2 0; 0 1/(0.3)^2];

    CA = [ca_x; ca_y];
    CB = [cb_x; cb_y];
    CC = [cc_x; cc_y];
    
    hAx = (1 - (X - CA)'*PA*(X - CA));
    hBx = (1 - (X - CB)'*PB*(X - CB));
    hCx = (1 - (X - CC)'*PC*(X - CC));
    h = [hAx, hBx, hCx];
        
    R = sqrt((x_new)^2 + (y_new)^2);
    theta = atan2(y_new, x_new);
    
    alpha_lp = (R - c)/sigma(2);
    beta = (theta - theta0)/sigma(1);
    
    paralpha = [(k^2*X(1))/(sigma(2)*R), ...
               (k*(k*X(2)+1))/(sigma(2)*R)];
          
    parbeta = [-(k*(k*X(2)+1))/(sigma(1)*R^2), ...
               (k^2*X(1))/(sigma(1)*R^2)];
           
    h_obs = (alpha_lp^p + beta^p)^(1/p) - abs(k); 
    
    A1 = 2*alpha(1)*(X - CA)'*PA + 2*alpha(2)*(X - CB)'*PB + 2*alpha(3)*(X - CC)'*PC;
    
    A2 = -(alpha_lp^p+beta^p)^((1-p)/p)*(alpha_lp^(p-1)*paralpha + ...
                                        beta^(p-1)*parbeta); 
                                    
    temp = -log(exp(-alpha(1)*hAx) + exp(-alpha(2)*hBx)+exp(-alpha(3)*hCx));
    
    if s == 0
        B1 = gamma * tanh(temp);
    else
        if task_num == 1
            B1 = gamma * tanh(temp) + hAx*(-sin(2*(t-t0))) + hBx*(sin(2*(t-t0)));
            
        elseif task_num == 2
            B1 = gamma * tanh(temp) + hBx*(-sin(2*(t-t0))) + hCx*(sin(2*(t-t0)));
            
        else
            B1 = gamma * tanh(temp) + hCx*(-sin(2*(t-t0)));
        end
        
    end
        
    B2 = gamma*h_obs^3;
    
    A = [A1; A2; 1 0; 0 1];
    B = [B1; B2; 10; 10];
    
    opts = optimoptions(@quadprog, 'Display', 'off', 'TolFun', 1e-5, 'TolCon', 1e-4);
    y = quadprog(H, [], A, B, [], [], [], [], [], opts);  

end