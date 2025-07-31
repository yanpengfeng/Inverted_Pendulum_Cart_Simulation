%% Check controllability and observability of the system at equilibrium

X = [0;0;0;0]; % states at equilibrium
U = 0; % effort at equilibrium
r = 1; % desired x_position 
p = get_parameters();
p.flag_ctrl = 1;

%调用函数来实现倒立摆数学模型
[A,B,C,D] = compute_cart_pole_linear_system(X,r,p); % Obtain Linear System
sys = ss(A,B,C,D)


Co = ctrb(A,B); % compute controllability matrix

unco = length(X) - rank(Co); % number of uncontrollable states
%运用x的长度减去可控性矩阵的秩来实现能控性判断

if unco == 0
    disp('The System is controllable at equilibrium');
else
    disp('The System is NOT controllable at equilibrium');
end


Ob = obsv(A,C);

unob = length(A) - rank(Ob);
%运用A矩阵行列的最大值作为参数来减去能观性矩阵的秩来实现能观性判断
if unob == 0
   disp('The System is observable at equilibrium');
else
    disp('The System is NOT observable at equilibrium');
end


