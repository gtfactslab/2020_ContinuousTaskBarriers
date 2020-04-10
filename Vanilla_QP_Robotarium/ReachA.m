function [h, y] = ReachA(X)
    
    gamma = 10;
    
    H = [1 0; 0 1];

    ca_x = 0.8; ca_y = 0.4; 
    cb_x = -0.2; cb_y = -0.6;
    
    PA = [1/(0.3)^2 0; 0 1/(0.2)^2];    
    PB = [1/(0.35)^2 0; 0 1/(0.1)^2];

    CA = [ca_x; ca_y];
    CB = [cb_x; cb_y];
    
    hAx = (1 - (X - CA)'*PA*(X - CA));
    hBx = (1 - (X - CB)'*PB*(X - CB));
    h = [hAx, hBx];

    A1 = 2*(X - CA)'*PA;
    B1 = gamma * sign(hAx);
    
    A = [A1; 1 0; 0 1];
    B = [B1; 10; 10];
        
    y = quadprog(H, [], A, B, [], [], [], []);  

end