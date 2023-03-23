function cinematica3DOF=CDBrazo3DOF(xu,yu,zu,psi,l1,l2,l3,a,b,c,q1,q2,q3)

hx = xu + a*cos(psi) - b*sin(psi) + cos(psi + q1)*(l3*cos(q2 + q3) + l2*cos(q2));
hy = yu + a*sin(psi) + b*cos(psi) + sin(psi + q1)*(l3*cos(q2 + q3) + l2*cos(q2));
hz = zu + c  - l1 - l3*sin(q2 + q3) - l2*sin(q2);

cinematica3DOF = [hx;hy;hz];
return