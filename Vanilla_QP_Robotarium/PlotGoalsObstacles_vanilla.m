function PlotGoalsObstacles_vanilla(P1, P2, c1, c2, c3, c4)

    xlabel('$x_{1}$', 'interpreter', 'latex')
    ylabel('$x_{2}$', 'interpreter', 'latex')
    hold on
    
    plot_ellipse(P1, c1, c2, 'b', '-');
    hold on
    
    plot_ellipse(P2, c3, c4, 'b', '-');
    hold on
 
end