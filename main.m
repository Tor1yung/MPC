clc;
clear;

%%%param and init
N = 7;  %预测步数
n = 3;  %状态维数
p = 3;  %控制器维数
% A = rand(n,n);
A = [0.237598887672087,0.466312220637624,0.765284866775821;0.817570881268703,0.951536218081585,0.574533590173997;0.405828714806259,0.965005305843558,0.915925025450222];
% B = rand(n,p);
B = [0.495432408790360,0.296436106009726,0.0689778462108583;0.166012377080293,0.558298491530906,0.166784582313363;0.325997669971674,0.0674768030510901,0.947438262628722];
C = []; % n*(N+1),p*N;
M = zeros(n*(N+1),1);
% Q = ones(n,n)*10;
% R = ones(p,p)*5;
% F = ones(n,n);
Q = eye(n,n)*100; 
R = eye(p,p)*0.01;
F = eye(n,n)*10;
Q_bar = []; % n*(N+1) , n*(N+1)
R_bar = []; % p*N , p*N

%约束  aX≤b,这里主要是约束控制器大小
a1 = eye(p,p);
a2 = -1*eye(p,p);
z = zeros((N-1)*p,(N-1)*p);
a = [blkdiag(a1,z);blkdiag(a2,z)];  %a矩阵
% a1 = eye(N*p,N*p);
% a2 = -1*eye(N*p,N*p);
% a = [a1;a2];  %a矩阵

u_bound = 250;   %约束控制器在多少的幅值内
b = u_bound*ones(2*N*p,1);      %b矩阵


X = rand(n,1)*500;   %初始化状态量
% X = [9.45174113109401;2.08934922426023;7.09281702710545];
X_1 = rand(n,1);
U = zeros(p,1);

X_whole = [];
yd_whole = [];
e_whole = [];
u_whole = [];

X_0 = X;
for i=1:1:N
Q_bar = blkdiag(Q_bar,Q);
R_bar = blkdiag(R_bar,R);
end
Q_bar = blkdiag(Q_bar,F);

for i=1:1:N
    row = [];
    for j=1:1:N
        if j <= i
            row = [row, A^(i-1-(j-1))*B];
        else
            row = [row,zeros(n,(N-j+1)*p)];
            break;
        end
    end
    
    C = [C;row];
end
C = [zeros(n,p*N);C];

for i=1:1:N+1
    if i==1
        M = ones(n,n);
    else
        M = [M;A^(i-1)];
    end
end


T = 10; 
t_delta = 0.01;

%开始循环
for t=1:1:T/t_delta

%     yd = 5;
    yd = [];
    ydd = [];
    for j=1:1:N+1
        yd = [yd;sin((t+j-1)*0.01)*100;sin((t+j-1)*0.01)*100;sin((t+j-1)*0.01)*100];    %ref
    end
    
    %保存ref用来画图
    ydd = [sin(t*0.01)*100;sin(t*0.01)*100;sin(t*0.01)*100];
    yd_whole(:,t) = ydd;

%%%controller
    U = MPC(X,yd,M,C,Q_bar,R_bar,a,b);
    u_whole = [u_whole,U(1:p)];
%%%system update
    X_1 = A*X+B*U(1:p);
    X = X_1;
    X_whole(:,t) = X;
    e = X - ydd;
    e_whole = [e_whole,e];
end



%%%画图

sequence = linspace(1, (T/t_delta), T/t_delta);

n_fol = p;
%画出控制器轨迹
figure(1);
hold on;
box on;
% plot(sequence,yd_whole ,'LineWidth',2.6);
legend('ref','NumColumns',1);
for i=1:1:n_fol
    e = reshape(u_whole(i,:),[],T/t_delta);
    plot(sequence,e ,'LineWidth',1.3);
end
% title("e");
legend({'ref','agent1','agent2','agent3'},'NumColumns',4);

%画出期望轨迹和跟踪轨迹
figure(2);
hold on;
box on;
plot(sequence,yd_whole(1,:) ,'b','LineWidth',2.6);
legend('ref','NumColumns',1);
for i=1:1:n_fol
    e = reshape(X_whole(i,:),[],T/t_delta);
    plot(sequence,e ,'LineWidth',1.3);
end
% title("e");
legend({'ref','agent1','agent2','agent3'},'NumColumns',4);


