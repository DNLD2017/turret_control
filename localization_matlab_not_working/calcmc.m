function rtr = calcmc(xt1,xt2,xtr1,xtr2,xvid1,xvid2)
dpsi1=xvid1(1);
dpsi2=xvid2(1);
dtheta1=xvid1(2);
dtheta2=xvid2(2);
psi1 = xt1(1)+dpsi1;
psi2 = xt2(1)+dpsi2;
theta1 = xt1(2)+dtheta1;
theta2 = xt2(2)+dtheta2;
x1=xtr1(1);
y1=xtr1(2);
z1=xtr1(3);
x2=xtr2(1);
y2=xtr2(2);
z2=xtr2(3);

A1=tan(psi1);
A2=-tan(theta1)/cos(psi1);
A3=tan(psi2);
A4=-tan(theta2)/cos(psi2);
B1=-1;
B2=-1;
B3=-1;
B4=-1;
C1=A1*x1-y1;
C2=A2*x1-z1;
C3=A3*x2-y2;
C4=A4*x2-z2;

M = [A1 B1 0;
       A2 0 B2;
       A3 B3 0;
       A4 0 B4];
rtr = [1 0 0;0 1 0;0 0 -1]*((M'*M)\M'*[C1;C2;C3;C4]);
end