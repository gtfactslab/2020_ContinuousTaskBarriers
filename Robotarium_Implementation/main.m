% Description: This is code to execute a sequence of three reachability
%              tasks, while avoiding an obstacle. The goal regions are
%              shown in green, the Lp function based obstacle is shown in
%              blue, and the trajectory of the robot is the dashed line
%
% NOTE:        Please run the "init.m" file prior to executing this code.
%
% Authors:     Mohit Srinivasan, Cesar Santoyo, and Samuel Coogan
% Date:        04/10/2020

%% Robotarium declarations
clear all;
close all;
clc;

N = 1;
iterations = 2000;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_pos_controller = create_si_position_controller();
si_to_uni_dyn = create_si_to_uni_mapping();
automatic_parking_controller = create_automatic_parking_controller();

num_tasks = 3;

%% Initialization of system parameters
X = [];
u = [];
u0 = [0; 0];
X0 = [-1; 1];
u = [u, u0];    
X = [X, X0];
dt = 1/30;
h = -1*ones(1, num_tasks);
i = 1;
U = [];
U_control = [];
t = 0;
T = [];
T_task = zeros(1, num_tasks);
Alpha = [];
tol = 0.01;
plot_flag = 1;

%% Plot the environment
c1 = 0.8; c2 = 0.4; c3 = -0.2; c4 = -0.6; c5 = -1; c6 = 0.2;
P1 = [1/(0.3)^2 0; 0 1/(0.2)^2];
P2 = [1/(0.35)^2 0; 0 1/(0.1)^2];
P3 = [1/(0.2)^2 0; 0 1/(0.3)^2];
PlotGoalsObstacles(P1, P2, P3, c1, c2, c3, c4, c5, c6);
hold on
init = [1.2; 0.8];

%% Solve the QP
for iter = 0:2000
    
   x = r.get_poses();

   dxi(:, 1) = [0 ; 0];

   dxu = automatic_parking_controller(x, [init; 0]);     
   r.set_velocities(1:N, dxu);
   r.step();
       
end

Plt_data1 = [];
Plt_data1 = [Plt_data1; x(1,1); x(2,1)];
p1 = plot(Plt_data1(1), Plt_data1(2), 'k-.', 'LineWidth', 3);
drawnow

for i = 1:num_tasks
    
    s = 0;
    
    while( h(i) < 0 )
        
        x = r.get_poses();
        alpha = compute_alphas(t, T_task(i), i, s);
        Alpha = [Alpha, alpha'];
        [barr, u] = ReachAB(x(1:2,:), alpha, t, T_task(i), s, i);
        U_control = [U_control, u];
        U = [U, norm(u, 2)];
        h = barr;
        
        dx = si_to_uni_dyn(u, x);
        r.set_velocities(1:N, dx);
        r.step();

        Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
        p1.XData = Plt_data1(1,:);
        p1.YData = Plt_data1(2,:);
        
        t = t + dt;
        T = [T, t];

    end
        
    T_task(i) = t;
    s = 1;
    
    if i == 1
        while alpha(1) > tol && alpha(2) < 1-tol
            x = r.get_poses();
            alpha = compute_alphas(t, T_task(i), i, s);
            [barr, u] = ReachAB(x(1:2,:), alpha, t, T_task(i), s, i);
            U_control = [U_control, u];
            U = [U, norm(u, 2)];
            h = barr;

            dx = si_to_uni_dyn(u, x);
            r.set_velocities(1:N, dx);
            r.step();

            Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
            p1.XData = Plt_data1(1,:);
            p1.YData = Plt_data1(2,:);

            Alpha = [Alpha, alpha'];
            t = t + dt;
            T = [T, t];
            
        end
        if alpha(1) < tol && alpha(2) > 1-tol
            alpha(1) = 0; alpha(2) = 1; alpha(3) = 0;
        end

    elseif i == 2
        while alpha(2) > tol && alpha(3) < 1-tol
            x = r.get_poses();
            alpha = compute_alphas(t, T_task(i), i, s);
            [barr, u] = ReachAB(x(1:2,:), alpha, t, T_task(i), s, i);
            U_control = [U_control, u];
            U = [U, norm(u, 2)];
            h = barr;

            dx = si_to_uni_dyn(u, x);
            r.set_velocities(1:N, dx);
            r.step();

            Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
            p1.XData = Plt_data1(1,:);
            p1.YData = Plt_data1(2,:);

            Alpha = [Alpha, alpha'];
            t = t + dt;
            T = [T, t];
            
        end
        if alpha(2) < tol && alpha(3) > 1-tol
            alpha(1) = 0; alpha(2) = 0; alpha(3) = 1;
        end

    else
        break
        
    end

end

t0 = T(end);
save('Exp_data.mat', 'U')
save('Exp_data2.mat', 'U_control')
r.debug();

%% Visualizations
if plot_flag == 1
    figure(2)
    plot(T, U,'LineWidth', 1.5)
    grid on
    grid minor
    set(gca,'TickLabelInterpreter','latex','FontSize',16);
    set(gcf,'color','w','Position',[1 41 1440 764]);
    xlabel('$t$','Interpreter','latex')
    ylabel('$u(t)$','Interpreter','latex')

    figure(3)
    subplot(3,1,1)
    plot(T, Alpha(1,:),'LineWidth', 1.5)
    grid on
    grid minor
    set(gca,'TickLabelInterpreter','latex','FontSize',16);
    xlabel('$t$','Interpreter','latex')
    ylabel('$\alpha_A(t)$','Interpreter','latex')

    subplot(3,1,2)
    plot(T, Alpha(2,:),'LineWidth', 1.5)
    grid on
    grid minor
    set(gca,'TickLabelInterpreter','latex','FontSize',16);
    xlabel('$t$','Interpreter','latex')
    ylabel('$\alpha_B(t)$','Interpreter','latex')

    subplot(3,1,3)
    plot(T, Alpha(3,:),'LineWidth', 1.5)
    set(gca,'TickLabelInterpreter','latex','FontSize',16);
    grid on
    grid minor
    xlabel('$t$','Interpreter','latex')
    ylabel('$\alpha_C(t)$','Interpreter','latex')
    set(gcf,'color','w','Position',[1 41 1440 764]);
end