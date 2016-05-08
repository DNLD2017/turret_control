function E = mat_euler(psi, theta, phi)
E = [cos(theta)*cos(psi) -cos(phi)*sin(psi)+sin(theta)*cos(psi)*sin(phi) sin(psi)*sin(phi)+sin(theta)*cos(psi)*cos(phi);
    cos(theta)*sin(psi) cos(psi)*cos(phi)+sin(theta)*sin(psi)*sin(phi) -cos(psi)*sin(phi)+sin(theta)*cos(phi)*sin(psi);
    -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
end