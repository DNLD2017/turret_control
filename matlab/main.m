clear all; close all; clc;
% Position :: x y z
xdr0= [-50 50 50]'; % Position initiale du drone
xtr1=[0 0 0]'; % Position initiale en 3d de la tourelle 1
xtr2=[100 0 0]'; % Position initiale en 3d de la tourelle 2
% Attitude :: cap, tangage, vitesse cap, vitesse tangage
x1=[0 0 0 0]'; % Attitude initiale de la tourelle 1
x2=[0 0 0 0]'; % Attitude initiale de la tourelle 2
% Vecteurs d'erreurs intégrales et dérivées
errI1=[0 0 0 0]';
errI2=[0 0 0 0]';
errD1=[0 0 0 0]';
errD2=[0 0 0 0]';

t=0;
k=1;
pas = 20;
dt=0.001;
duree = 5;
n=duree/dt/pas;
% Vecteurs de stockage
rtrs = zeros(3,n);
xdrs = zeros(3,n);
xvid1s = zeros(2,n);
xvid2s = zeros(2,n);
ts = zeros(1,n);

while (t<=duree)
   xdr = xdr0 + [cos(pi*t);sin(pi*t);1*cos(pi*t)];
   xvid1=camera(xdr,xtr1,x1); % Erreur angulaire mesurée par la caméra 1
   [u1,errI1,errD1]=correc(xvid1,errI1,errD1,dt); % Commande envoyée aux servomoteurs de la tourelle 1
   xvid2=camera(xdr,xtr2,x2); % Erreur angulaire mesurée par la caméra 2
   [u2,errI2,errD2]=correc(xvid2,errI2,errD2,dt); % Commande envoyée aux servomoteurs de la tourelle 2
   
   %rtr = calcmc(x1,x2,xtr1,xtr2,xvid1,xvid2); % Position du drone calculée théoriquement
   rtr = calcmc(x1,x2,xtr1,xtr2,xvid1,xvid2); % Position du drone calculée par la méthode des moindres carrés
   x1=x1+turret(x1,u1)*dt; % Nouvelle attitude de la tourelle 1
   x2=x2+turret(x2,u2)*dt; % Nouvelle attitude de la tourelle 2
   if(mod(k,pas)==0)
        rtrs(:,k) = rtr;
        xdrs(:,k) = xdr;
        xvid1s(:,k) = 180/pi*xvid1;
        xvid2s(:,k) = 180/pi*xvid2;
        ts(:,k) = t;
        t
   end
   k=k+1;
   t=t+dt;
end
figure;
draw(rtrs,xdrs,xvid1s,xvid2s,ts); % Grahique position réelle bleue - Position calculée rouge