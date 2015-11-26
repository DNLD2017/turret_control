function rtr = calcp(x1,x2,xtr1,xtr2,xvid1,xvid2)
dpsi1=xvid1(1);
dpsi2=xvid2(1);
dtheta2=xvid2(2);
psi1 = x1(1)+dpsi1;
psi2 = x2(1)+dpsi2;
theta2 = x2(2)+dtheta2;
x01=xtr1(1);
x02=xtr1(2);
x11=xtr2(1);
x12=xtr2(2);
x13=xtr2(3);
dx=x11-x01;
dy=x12-x02;
% Distance à la deuxième tourelle
d2=(dy-tan(psi1)*dx)/(tan(psi1)*cos(theta2)*cos(psi2)-cos(theta2)*sin(psi2));
% Position calculée du drone
x = d2*cos(theta2)*cos(psi2)+x11;
y = d2*cos(theta2)*sin(psi2)+x12;
z = d2*sin(theta2)+x13;
rtr=[x;y;z];
end