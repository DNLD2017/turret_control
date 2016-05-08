function [u,errI,errD] = correc(xvid,errI,errD,dt)
% Correcteur prenant en entrée les données caméra et ressortant la commande
% en vitesse pour chaque servomoteur d'une tourelle
kp=50;
ki=10000000000000000000;
kd=0;
u=kp*[0 0 xvid(1) xvid(2)]'+1/ki*errI*dt+kd*([0 0 xvid(1) xvid(2)]'-errD);
errI=errI+[0 0 xvid(1) xvid(2)]';
errD=[0 0 xvid(1) xvid(2)]';
end