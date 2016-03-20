function [pos,G1] = fusion(x0,old, new,G0)
    
    psi1 = old(1);
    theta1 = old(2);
    phi1 = old(3);
    x1 = old(4);
    y1 = old(5);
    z1 = old(6);
    epsi1 = old(7);
    etheta1 = old(8);
    
    psi2 = new(1);
    theta2 = new(2);
    phi2 = new(3);
    x2 = new(4);
    y2 = new(5);
    z2 = new(6);
    epsi2 = new(7);
    etheta2 = new(8);
    
    %Vecteurs pointant vers le drone depuis les tourelles
    k1=mat_euler(psi1, theta1, phi1)*mat_euler(epsi1, etheta1, 0)*[1 0 0]';
    k2=mat_euler(psi2, theta2, phi2)*mat_euler(epsi2, etheta2, 0)*[1 0 0]';
    
    % Affichage des vecteurs pointants
    plot([x1 20*k1(1)],[y1 20*k1(2)],'r');
    plot([x2 20*k2(1)+x2],[y2 20*k2(2)+y2],'g');
    
    C = [1 0 0 -k1(1) 0;
         0 1 0 -k1(2) 0;
         0 0 1 -k1(3) 0;
         1 0 0 0 -k2(1);
         0 1 0 0 -k2(2);
         0 0 1 0 -k2(3)];
    
    A = zeros(5);
    u = zeros(5,1);
    y = [x1 y1 z1 x2 y2 z2]';
    G_alpha = -diag([0.01 0.01 0.01 0 0]);
    G_beta = 10000*[0.1 0 0 0 0 0;
              0 0.1 0 0 0 0;
              0 0 0.1 0 0 0;
              0 0 0 0.1 0 0;
              0 0 0 0 0.1 0;
              0 0 0 0 0 0.1];
    
    [pos,G1] = kalman(x0,G0,u,y,G_alpha,G_beta,A,C);
end