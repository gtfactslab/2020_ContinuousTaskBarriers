function PlotGoalsObstacles(P1, P2, P3, c1, c2, c3, c4, c5, c6)

    xlabel('$x_{1}$', 'interpreter', 'latex')
    ylabel('$x_{2}$', 'interpreter', 'latex')
    hold on
    
    plot_ellipse(P1, c1, c2, 'g', '-');
    hold on
    
    plot_ellipse(P2, c3, c4, 'g', '-');
    hold on

    plot_ellipse(P3, c5, c6, 'g', '-');
    hold on

    p = 6;
    sigma = [0.7, 0.2];
    theta_k = pi/2;
    k = theta_k/(2*sigma(1));
    c = 1;
    theta0 = sign(k)*pi/2;
    [x,y] = meshgrid(-1.6:0.001:1.6, -1:0.001:1);

    x_new = k.*x;
    y_new = k.*y + 1;

    R = ((x_new).^2 + (y_new).^2).^(1/2);
    theta = atan2(y_new, x_new);

    alpha = abs(R - c)/sigma(2);
    beta = abs(theta - theta0)/sigma(1);

    Hg = (alpha.^(p) + beta.^(p)).^(1/p) - abs(k);

    contour(x, y, Hg,[0, 0], 'DisplayName', 'Barrier Function Level Sets', 'LineWidth', 3, 'ShowText', 'off');
    hold on
 
end