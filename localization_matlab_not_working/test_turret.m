clear all; close all; clc;
% Attitude :: cap, tangage, vitesse cap, vitesse tangage
x1=[0 0 0 0]'; % Attitude initiale de la tourelle 1
u1 = [0;0;10;10];
k=0;
t=0;
dt=0.001;
duree = 2;
figure;
while (t<duree)
    x1=x1+turret(x1,u1)*dt; % Nouvelle attitude de la tourelle 1
    if(mod(k,10)==0)
        subplot(1,2,1);
        hold on;
        plot(t,x1(3),'.b');
        title('Vitesse lacet / temps');
        subplot(1,2,2);
        hold on;
        plot(t,x1(4),'.r');
        title('Vitesse tangage / temps');
        drawnow;
    end
    t=t+dt;
    k=k+1;
end