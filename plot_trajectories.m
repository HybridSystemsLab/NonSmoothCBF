%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Example: CBF with rectangle obstacle with forward Euler
% Name: Masoumeh Ghanbarpour
% Plot Trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% parameters

% Font size
fontsize = 14;
% Colors
cc= [0 0.4470 0.7410; 
    0.9290 0.6940 0.1250;
    0.4940 0.1840 0.5560; 
    0.8500 0.3250 0.0980;
    0.6510    0.6510    0.6510%0.6350 0.0780 0.1840;
     0.4275    0.7294    0.8392%0.3010 0.7450 0.9330;
    0.3922    0.8314    0.0745];

% Symboles
symbs = {':' '--' '-.' ':' '--' ':' '-.' };

%% Plot

figure
set(0,'defaulttextinterpreter','latex')

% plot safe area
L_p = obst.L + obst.Rs;
W_p = obst.W + obst.Rs;
vertices_plus = [cos(obst.theta), -sin(obst.theta), obst.zo(1); sin(obst.theta), cos(obst.theta), obst.zo(2); 0,0,1]*[-L_p,-L_p,L_p,L_p;-W_p,W_p,W_p,-W_p;1,1,1,1];
fill(vertices_plus(1,1:4),vertices_plus(2,1:4),[ 0.9216    0.9216    0.4314],'LineStyle','none');%[254/255,235/255,117/255])
hold all

% plot obstacle
fill(obst.vertices(1,1:4),obst.vertices(2,1:4),[0.4660 0.6740 0.1880],'LineStyle','none') %[0.4660 0.6740 0.1880])
text(obst.zo(1),obst.zo(2), 'OBSTACLE','fontsize',12,'HorizontalAlignment','center','VerticalAlignment', 'middle','rotation',45)

% plot the target point
plot(xt(1),xt(2),'o','LineWidth',1, 'MarkerSize',10,'color' , [.5 0 .5])
hold on 
for i=1:length(X)
    x0 = X0(i,:);
    
    % plot the initial point
    plot(x0(1),x0(2),'*','LineWidth',1, 'MarkerSize',10,'color', cc(i,:))
    hold on
    
    set(gca,'fontsize',fontsize-2)
    hold all
    box on
    grid
    axis equal


    % Plot trajectory
    x = X{i};
    u = U{i};
    set(0,'defaulttextinterpreter','latex')
    plot(x(1,:),x(2,:),symbs{i},'color',cc(i,:),'LineWidth',2); 
    hold on
    sc = 7;

    xlabel('$x_1$','FontSize',24,'FontWeight','bold')
    ylabel('$x_2$','FontSize',24,'FontWeight','bold')
    grid on
    axis([-1 8 -3 3])
end
plot(-.7,-2.2,'*','LineWidth',1, 'MarkerSize',10, 'color',[0 0.4470 0.7410])
text(-.5,-2.2, 'START POINT','fontsize',12,'HorizontalAlignment','left','VerticalAlignment', 'middle')
plot(-.7,-2.8,'o','LineWidth',1, 'MarkerSize',10,'color' , [.5 0 .5])
text(-.5,-2.8, 'DESTINATION','fontsize',12,'HorizontalAlignment','left','VerticalAlignment', 'middle')


