function u = u_func(x, xt, obst, dyn)

%%%%%%%%%%%%%%%%%%
    % u_func
    % input: x, xt, obst, dyn
        % x: position
        % xt: target point
        % obst: obstacle information
        % dyn: dynamic of system
   
        
    % output: u
        % u : control
        
    % functions: compute control input
%%%%%%%%%%%%%%%%%%

%% Parameters

p = 20;
gamma = 5;
alpha = 0.01;
M = 1000;

%% Barrier function

[b_value,gradient_b] = Barrier(x, obst);
   
%% Lyapunov function

V = @(x) 0.5*norm(x-xt)^2;

%% Constraints

% U
A = [1 0;0 1;-1 0;0 -1];
b = 5*[1;1;1;1];

% Safety
idx = find(abs(b_value - min(b_value)) <= 0);
idx_alpha = find(abs(b_value - min(b_value)) <= alpha);

temp_b = zeros(length(idx),2);
f = dyn.A * x;
temp_A = zeros(length(idx),1);
for i =1:length(idx)
    temp_b(i,:) = gradient_b(idx(i),:) * dyn.b;
    temp_A(i) = -gradient_b(idx(i),:) * f;
end
A = [A;temp_b];
b = [b ; temp_A + max(0,-M*min(b_value))*ones(length(idx),1)];

if ~isequal(idx, idx_alpha) 
    mm = intersect(idx_alpha,idx);
    md = setdiff(idx_alpha,idx);
    phi = pi * abs(b_value(md) - min(b_value))/(2 * alpha);
    vec = sin(phi)* gradient_b(mm,:) + cos(phi) * gradient_b(md,:);
    temp_b2 = vec * dyn.b;
    A = [A;temp_b2];
    temp_A2 = -vec * f;
    
    b = [b ;temp_A2+max(0,-M*min(b_value))];
end

%% Optimization

    n =2;
    cvx_begin quiet
         variable u(n)
         variable delta(1)
         minimize( 0.5*u'* u+ 0.5* p * delta)
         subject to
         (x-xt)'* dyn.b*u <= -(x-xt)'*f- gamma *V(x) +delta
         A * u <= b;
    cvx_end
