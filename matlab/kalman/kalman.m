function [xup,Gup]=kalman(x0,G0,u,y,G_alpha,G_beta,A,C)
if(isempty(y)),
    n=length(x0);
    y=eye(0,1); G_beta=eye(0,0); C=eye(0,n);
end
S=C*G0*C'+G_beta;
K=G0*C'*inv(S);
y_tilde=y-C*x0;
xup=x0+K*y_tilde;
Gup=G0-K*C*G0;
x1=A*xup+u;
G1=A*Gup*A'+G_alpha;
end