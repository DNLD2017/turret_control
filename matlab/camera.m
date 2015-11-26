function xvid = camera(xdr,xtr,x)
% Simulation des mesures de la caméra, qui renvoie une erreur angulaire par
% différence entre l'attitude idéale et l'attitude courante.
psid = atan2((xdr(2)-xtr(2)),(xdr(1)-xtr(1)));
thetad = atan2((xdr(3)-xtr(3)),(cos(psid)*(xdr(1)-xtr(1))+sin(psid)*(xdr(2)-xtr(2))));
xvid=[psid-x(1);thetad-x(2)]+5*pi/180*rand(2,1);
end