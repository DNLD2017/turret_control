clear all; close all; clc;
% Position :: x y z
xdr0= [-50 50 50]'; % Position initiale du drone
xtr1=[0 0 0]'; % Position initiale en 3d de la tourelle 1
xtr2=[100 0 0]'; % Position initiale en 3d de la tourelle 2
% Attitude :: cap, tangage, vitesse cap, vitesse tangage
x1=[pi/2 0]'; % Attitude initiale de la tourelle 1
x2=[pi/2 0]'; % Attitude initiale de la tourelle 2

rtr = [-50 50 50 0 0]';

t=0;
dt=0.1;
duree = 50;
G0 = 100000*diag(ones(5,1));
figure;
plot(rtr(1),rtr(2),'*black');
for t=0:dt:duree
    clf; axis([-100 100 -100 100]); axis square;hold on;
    xdr = xdr0+50*[cos(0.1*pi*t);sin(0.1*pi*t);0];
    xvid1=camera(xdr,xtr1,x1); % Erreur angulaire drone-image mesurée par la caméra 1
    xvid2=camera(xdr,xtr2,x2); % Erreur angulaire drone-image mesurée par la caméra 2
    
    old=[x1(1:2)' 0 xtr1' xvid1']';
    new=[x2(1:2)' 0 xtr2' xvid2']';
    % Fusion des données
    [rtr,G0] = fusion(rtr,old,new,G0);
    
    f1=xvid1*5*pi/3; % Commande envoyée aux servomoteurs de la tourelle 1
    f2=xvid2*5*pi/3; % Commande envoyée aux servomoteurs de la tourelle 2
    x1=x1+f1*dt; % Nouvelle attitude de la tourelle 1
    x2=x2+f1*dt; % Nouvelle attitude de la tourelle 2
    
    t=t+dt;
    
    % Affichage
    plot(xdr(1),xdr(2),'+g');
    plot(rtr(1),rtr(2),'.r');
    draw_ellipse(rtr(1:2),G0(1:2,1:2),0.999);
    drawnow();
end
% Le kalman ne fonctionne pas à cause des incertitudes sur les angles non prises en compte. Le système n'est pas linéaire.