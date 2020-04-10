function plot_ellipse(P, a, b, c, type)

      theta = 0:0.00001:2*pi;
      x = (1/sqrt(P(1,1)))*cos(theta) + a;
      y = (1/sqrt(P(2,2)))*sin(theta) + b;
      C = [x; y];
      x = C(1,:);
      y = C(2,:);
      plot(x, y, 'Color', c, 'LineStyle', type,'LineWidth', 3);
      hold on

end 