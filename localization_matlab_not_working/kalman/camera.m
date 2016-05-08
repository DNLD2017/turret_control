function xvid = camera(xdr,xtr,x)
% Simulation des mesures de la caméra, qui renvoie une erreur angulaire par
% différence entre l'attitude idéale et l'attitude courante.
k=(xdr-xtr)/norm(xdr-xtr);
A=mat_euler(x(1),x(2),0)\k;
epsi=atan2(A(2),A(1));
etheta=atan2(-A(3),A(1)*cos(epsi));
xvid=[epsi;etheta]+10*pi/180*rand(2,1)-5*pi/180;
end