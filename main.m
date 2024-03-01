clc;
clear;

%%%param and init
N = 5;  %预测步数
n = 3;  %状态维数
p = 3;  %控制器维数
A = rand(n,n);
% A = [7.65500016621438,0.911134636865350,5.46593114590323;1.88661976791491,5.76209380663007,4.25728841871188;2.87498173066131,6.83363243294653,6.44442781431336];
B = rand(n,p);
% B = [6.47617630172684;6.79016754093202;6.35786710514084];
C = []; % n*(N+1),p*N;
M = zeros(n*(N+1),1);
% Q = ones(n,n)*10;
% R = ones(p,p)*5;
% F = ones(n,n);
Q = eye(n,n)*100; 
R = eye(p,p)*0.0001;
F = eye(n,n)*10;
Q_bar = []; % n*(N+1) , n*(N+1)
R_bar = []; % p*N , p*N


X = rand(n,1)*10000;
% X = [9.45174113109401;2.08934922426023;7.09281702710545];
X_1 = rand(n,1);
U = zeros(p,1);

X_whole = [];
yd_whole = [];
e_whole = [];

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

for t=1:1:T/t_delta

%     yd = 5;
    yd = [];
    ydd = [];
    for j=1:1:N+1
        yd = [yd;sin((t+j-1)*0.01)*100;sin((t+j-1)*0.01)*100;sin((t+j-1)*0.01)*100];
    end
    ydd = [sin(t*0.01)*100;sin(t*0.01)*100;sin(t*0.01)*100];
    yd_whole(:,t) = ydd;

%%%controller
    U = MPC(X,yd,M,C,Q_bar,R_bar);
    
%%%system update
    X_1 = A*X+B*U(1:p);
    X = X_1;
    X_whole(:,t) = X;
    e = X - ydd;
    e_whole = [e_whole,e];
end



%%%paint

sequence = linspace(1, (T/t_delta), T/t_delta);
% time_range = [0, t_whole];
% mapped_time = (sequence - min(sequence)) / (max(sequence) - min(sequence)) * (time_range(2) - time_range(1)) + time_range(1);


n_fol = 3;
% figure(1);
hold on;
box on;
% plot(sequence,yd_whole ,'LineWidth',2.6);
for i=1:1:n_fol
    e = reshape(e_whole(i,:),[],T/t_delta);
    plot(sequence,e ,'LineWidth',1.3);
end
% title("e");
% legend({'agent1','agent2','agent3'},'NumColumns',3);
% % title("Number of Trigger");
% set(gca,'FontSize',16,'FontName','Times New Roman');
% % xlabel('t','FontSize',25);
% % ylabel('error','FontSize',25);
% txt = xlabel('Time(sec)','FontSize',25);
% set(txt,'Interpreter','none');
% txt = ylabel('$\mathcal{Z}_{2i}$','FontSize',25);
% set(txt,'Interpreter','latex');

