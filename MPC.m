

function U = MPC(x_k,yd,M,C,Q_bar,R_bar)

H = C'*Q_bar*C+R_bar;
% H = (H+H')/2;
E = (x_k'*M'-yd')*Q_bar*C;

U = quadprog(H,E);

end