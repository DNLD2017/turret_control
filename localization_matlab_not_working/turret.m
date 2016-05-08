function xdot = turret(x,u)
% Modélisation du comportement de la tourelle
% x :: cap, tangage, vitesse cap, vitesse tangage
k=50;
M=[0 0 1 0;
   0 0 0 1;
   0 0 -k 0;
   0 0 0 -k];
xdot=M*x+k*u+0.5*rand(4,1);
end