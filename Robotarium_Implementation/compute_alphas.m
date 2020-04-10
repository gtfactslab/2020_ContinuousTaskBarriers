function alpha = compute_alphas(t, T_task, i, s)
    
    alpha = zeros(1, 3);
    
    if s == 0
        alpha(i) = 1;
        
    else
        if i == 1
            alpha(1) = cos(t - T_task)^2;
            alpha(2) = sin(t - T_task)^2;
            alpha(3) = 0;
            
        elseif i == 2
            alpha(2) = cos(t - T_task)^2;
            alpha(1) = 0;
            alpha(3) = sin(t - T_task)^2;
            
        else
            alpha(3) = cos(t - T_task)^2;
            alpha(1) = 0;
            alpha(2) = 0;
        end
        
    end

end