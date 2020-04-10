% Description: This is code to execute a sequence of two reachability
%              tasks using the vanilla QP switching method
% NOTE:        Please run the "init.m" file from the Robotarium package 
%              prior to executing this code
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

num_tasks = 2;

%% Initialization of system parameters
X = [];
u = [];
u0 = [0; 0];
X0 = [-1; 1];
u = [u, u0];    
X = [X, X0];
dt = 0.001;
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
c1 = 0.8; c2 = 0.4; c3 = -0.2; c4 = -0.6;
P1 = [1/(0.3)^2 0; 0 1/(0.2)^2];
P2 = [1/(0.35)^2 0; 0 1/(0.1)^2];
PlotGoalsObstacles_vanilla(P1, P2, c1, c2, c3, c4);
hold on
init = [1.2; 0.8];

%% Solve the QP
for iter = 0:2000
    
   x = r.get_poses();

   dxi(:, 1) = [0 ; 0];

   dxu = automatic_parking_controller(x, [init; pi]);     
   r.set_velocities(1:N, dxu);
   r.step();
       
end

Plt_data1 = [];
Plt_data1 = [Plt_data1; x(1,1); x(2,1)];
p1 = plot(Plt_data1(1), Plt_data1(2), 'k-.', 'LineWidth', 3);
drawnow

while( h(1) < 0 )

    x = r.get_poses();
    [barr, u] = ReachA(x(1:2,:));
    U_control = [U_control, u];
    U = [U, norm(u, 2)];
    h(1) = barr(1);

    dx = si_to_uni_dyn(u, x);
    r.set_velocities(1:N, dx);
    r.step();

    Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
    p1.XData = Plt_data1(1,:);
    p1.YData = Plt_data1(2,:);

    t = t + dt;
    T = [T, t];

end

while( h(2) < 0 )

    x = r.get_poses();
    [barr, u] = ReachB(x(1:2,:));
    U_control = [U_control, u];
    U = [U, norm(u, 2)];
    h(2) = barr(2);

    dx = si_to_uni_dyn(u, x);
    r.set_velocities(1:N, dx);
    r.step();

    Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
    p1.XData = Plt_data1(1,:);
    p1.YData = Plt_data1(2,:);

    t = t + dt;
    T = [T, t];

end

r.debug();

%% Visualizations
if plot_flag == 1
    figure(2)
    plot(T, U,'LineWidth', 1.5)
    title('Control law $||u||_{2}$', 'interpreter','latex')
    grid on
    grid minor
    set(gca,'TickLabelInterpreter','latex','FontSize',16);
    set(gcf,'color','w','Position',[1 41 1440 764]);
    xlabel('$t$','Interpreter','latex')
    ylabel('$u(t)$','Interpreter','latex')

end