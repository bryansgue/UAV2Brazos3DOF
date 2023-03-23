function M = D_H(q,d,a,alpha)
% R1= cos(alpha)
% R1 = 
Rotz = [cos(q) -sin(q) 0 0; 
        sin(q) cos(q) 0 0 ; 
        0 0 1 0; 
        0 0 0 1];
TrasZ = [1 0 0 0; 
         0 1 0 0 ; 
         0 0 1 d; 
         0 0 0 1];
TrasX = [1 0 0 a;
         0 1 0 0 ; 
         0 0 1 0; 
         0 0 0 1];
Rotx = [1 0 0 0; 
        0 cos(sym(alpha)) -sin(sym(alpha)) 0 ; 
        0 sin(sym(alpha)) cos(sym(alpha)) 0; 
        0 0 0 1]; 
    


H = [cos(q) -cos(sym(alpha))*sin(q) sin(sym(alpha))*sin(q) a*cos(q);
     sin(q) cos(sym(alpha))*cos(q) -sin(sym(alpha))*cos(q) a*sin(q);
     0 sin(sym(alpha)) cos(sym(alpha)) d;
     0 0 0 1]   ;

% M = Rotz*TrasZ*TrasX*Rotx;

M = H;