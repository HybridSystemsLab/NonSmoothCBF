%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Example: CBF with rectangle obstacle with forward Euler
% Name: Masoumeh Ghanbarpour
% main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%% Parameters

%%% Obstacle
obst.zo = [5;0];        % center point of obstacle
obst.L = 1.2;           % Length of rectangle
obst.W = 0.5;           % Width of rectangle
obst.theta = 4*pi/11;   % Angle of rectangle

obst.Rs = 0.3;          % Safety distance

% Vertices of obstacle
obst.vertices = [cos(obst.theta), -sin(obst.theta), obst.zo(1); 
                 sin(obst.theta), cos(obst.theta), obst.zo(2); 
                 0,0,1] *[-obst.L,-obst.L,obst.L,obst.L;-obst.W,obst.W,obst.W,-obst.W;1,1,1,1];

%%% Initial points
X0 = [7,0; 
    6.5,-1;
    5,2;
    5.5,-1.8;
    6,0;
    6,2.5;
    7,2];

%%% Target point
xt = [0;0];


%%% Dynamics
dyn.A = [0, 1 ;
        -1,-1];
dyn.b = [1,0;
         0,1];
     
%%% Time span
h = 0.005;
tspan = 0:h:10;

%%% PLot while simulation
plot_while_simulation = 1;

%%% Font size
fontsize = 14;

%% Main

%%% plot prepration
if plot_while_simulation
    
    figure
    %%% plot safe area
    L_p = obst.L + obst.Rs;
    W_p = obst.W + obst.Rs;
    vertices_plus = [cos(obst.theta), -sin(obst.theta), obst.zo(1);
                     sin(obst.theta), cos(obst.theta), obst.zo(2); 
                     0,0,1] * [-L_p,-L_p,L_p,L_p;-W_p,W_p,W_p,-W_p;1,1,1,1];
    fill(vertices_plus(1,1:4),vertices_plus(2,1:4),[254/255,235/255,117/255])
    hold all

    %%% plot obstacle
    plot([obst.vertices(1,1:4),obst.vertices(1,1)],[obst.vertices(2,1:4),obst.vertices(2,1)],'color', [17 17 17]/255,'LineWidth', 2)
    fill(obst.vertices(1,1:4),obst.vertices(2,1:4),[0.4660 0.6740 0.1880])

    %%% plot the target point
    plot(xt(1),xt(2),'o','LineWidth',1, 'MarkerSize',10)
    hold on 
    text(xt(1)-.4,xt(2)+0.4, 'DESTINATION','fontsize',fontsize)
    hold on 


    set(gca,'fontsize',fontsize-2)
    hold all
    box on
    grid
    axis equal
    xlim([-0.7 9.5])
end

%% Simulation
N = length(tspan);
for j =1: size(X0,1)
    x0 = X0(j,:);
    x = zeros( numel(x0), N );
    u = zeros( 2, N );
    x(:,1) = x0;
    u(:,1) = [0;0];
    for i=1:N-1
        u(:,i+1) = u_func(x(:,i),xt,obst,dyn);
        x(:,i+1) = x(:,i) + h * (dyn.A* x(:,i) + dyn.b * u(:,i+1));

        %%% Animation part
        if plot_while_simulation
            hold on
            if (i>1)
                delete(ff1)
                %delete(ff2)
            end
            ff1 = plot(x(1,i+1),x(2,i+1),'bs','LineWidth', 1);
            ff2 = quiver(x(1,i+1),x(2,i+1),u(1,i+1)/norm(u(:,i+1)),u(2,i+1)/norm(u(:,i+1)),'LineWidth', 1,'MaxHeadSize', 1);
            set(ff2,'AutoScale','on', 'AutoScaleFactor', 0.5)
            pause(0.05)
        end
        if norm(x(:,i+1)-xt) <7e-2
            break
        end
    end
    X{j} = x(:,1:i);
    U{j} = u(:,1:i); 
    T{j} = tspan(1:i);
end
%% PLot

plot_trajectories;