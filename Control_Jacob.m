
function [VMref] = Control_Jacob(l2_1,l3_1,l2_2,l3_2,a_1,b_1,a_2,b_2,q1_1,q2_1,q3_1,q1_2,q2_2,q3_2,psi,hx_1,hy_1,hz_1,hx_2,hy_2,hz_2,xd_1,yd_1,zd_1,xd_2,yd_2,zd_2,psid,xdp_1,ydp_1,zdp_1,xdp_2,ydp_2,zdp_2,psidp) 
%% 1) Definicion de la jacobiana del sistema
  
    J11 = cos(psi);
    J12 = -sin(psi);
    J13 = 0;
    J14 = - sin(psi + q1_1)*(l3_1*cos(q2_1 + q3_1) + l2_1*cos(q2_1)) - b_1*cos(psi) - a_1*sin(psi);
    J15 = -sin(psi + q1_1)*(l3_1*cos(q2_1 + q3_1) + l2_1*cos(q2_1));
    J16 = -cos(psi + q1_1)*(l3_1*sin(q2_1 + q3_1) + l2_1*sin(q2_1));
    J17 = -l3_1*cos(psi + q1_1)*sin(q2_1 + q3_1);
    J18 = 0;
    J19 = 0;
    J110 = 0;

    J21 = sin(psi);
    J22 = cos(psi);
    J23 = 0;
    J24 = cos(psi + q1_1)*(l3_1*cos(q2_1 + q3_1) + l2_1*cos(q2_1)) + a_1*cos(psi) - b_1*sin(psi);
    J25 = cos(psi + q1_1)*(l3_1*cos(q2_1 + q3_1) + l2_1*cos(q2_1));
    J26 = -sin(psi + q1_1)*(l3_1*sin(q2_1 + q3_1) + l2_1*sin(q2_1));
    J27 = -l3_1*sin(psi + q1_1)*sin(q2_1 + q3_1);
    J28 = 0; 
    J29 = 0;
    J210 = 0;

    J31 =0;
    J32 =0;
    J33 =1;
    J34 =0;
    J35 =0;
    J36 = - l3_1*cos(q2_1 + q3_1) - l2_1*cos(q2_1);
    J37 = -l3_1*cos(q2_1 + q3_1);
    J38 = 0;    
    J39 = 0;
    J310 = 0;
 
    J41 = cos(psi);
    J42 = -sin(psi);
    J43 = 0;
    J44 = - sin(psi + q1_2)*(l3_2*cos(q2_2 + q3_2) + l2_2*cos(q2_2)) - b_2*cos(psi) - a_2*sin(psi);
    J45 = 0;
    J46 = 0;
    J47 = 0;
    J48 = -sin(psi + q1_2)*(l3_2*cos(q2_2 + q3_2) + l2_2*cos(q2_2));      
    J49 = -cos(psi + q1_2)*(l3_2*sin(q2_2 + q3_2) + l2_2*sin(q2_2));
    J410 = -l3_2*cos(psi + q1_2)*sin(q2_2 + q3_2);

    J51 = sin(psi);
    J52 = cos(psi);
    J53 = 0;
    J54 = cos(psi + q1_2)*(l3_2*cos(q2_2 + q3_2) + l2_2*cos(q2_2)) + a_2*cos(psi) - b_2*sin(psi);
    J55 = 0;
    J56 = 0;
    J57 = 0;
    J58 = cos(psi + q1_2)*(l3_2*cos(q2_2 + q3_2) + l2_2*cos(q2_2));
    J59 = -sin(psi + q1_2)*(l3_2*sin(q2_2 + q3_2) + l2_2*sin(q2_2));
    J510 = -l3_2*sin(psi + q1_2)*sin(q2_2 + q3_2);
   
    J61 = 0;
    J62 = 0;
    J63 = 1;
    J64 = 0;
    J65 = 0;
    J66 = 0;
    J67 = 0;
    J68 = 0;
    J69 = - l3_2*cos(q2_2 + q3_2) - l2_2*cos(q2_2);
    J610 = -l3_2*cos(q2_2 + q3_2);
    
    J71 = 0;
    J72 = 0;
    J73 = 0;
    J74 = 1;
    J75 = 0;
    J76 = 0;
    J77 = 0;
    J78 = 0;
    J79 = 0;
    J710 = 0;
    
    

% 2) Matriz jacobiana
  J = [J11 J12 J13 J14 J15 J16 J17 J18 J19 J110;...
       J21 J22 J23 J24 J25 J26 J27 J28 J29 J210;...
       J31 J32 J33 J34 J35 J36 J37 J38 J39 J310;...
       J41 J42 J43 J44 J45 J46 J47 J48 J49 J410;...
       J51 J52 J53 J54 J55 J56 J57 J58 J59 J510;...
       J61 J62 J63 J64 J65 J66 J67 J68 J69 J610;
       J71 J72 J73 J74 J75 J76 J77 J78 J79 J710];
   
   %% 3) Calculos del Error
  % Brazo 1
  hxe_1= xd_1 - hx_1;
  hye_1= yd_1 - hy_1;
  hze_1= zd_1 - hz_1;
  %psie_1= Angulo(psid_1-psi);
  % Brazo 2
  hxe_2= xd_2 - hx_2;
  hye_2= yd_2 - hy_2;
  hze_2= zd_2 - hz_2;
  %psie_2= Angulo(psid_2-psi);
  % Rotacion del UAV
  psie= Angulo(psid-psi);
  % Vector de error
  he= [hxe_1 hye_1 hze_1 hxe_2 hye_2 hze_2 psie]';
  
%% c)Matriz de Ganancia
  W = diag(1.25*[1 1 1 1 1 1 1]);
  R = diag([1 1 1 1 3 2 1 3 2 1]);
  
%% d)Vector Nulo
    q1_1dm = 0; q2_1dm = 90*pi/180; q3_1dm = -30*pi/180; 
    q1_2dm = 0; q2_2dm = 90*pi/180; q3_2dm = -30*pi/180; 
    n = [0;...
         0;...
         0;...
         0;...
         q1_1dm - q1_1;...
         q2_1dm - q2_1;...
         q3_1dm - q3_1;...
         q1_2dm - q1_2;...
         q2_2dm - q2_2;...
         q3_2dm - q3_2];
    
% 6) Calculo del vector de espacio nulo
   EspNulo= (eye(10)-pinv(J)*J)*R*tanh(n);
  
% 7) Ley de control completa,  solucion = [u omega qpunto1 qpunto2]   
  hdp = [xdp_1 ydp_1 zdp_1 xdp_2 ydp_2 zdp_2 psidp]';
  
   %VMref = pinv(J)*(W*tanh(0.96*he))+EspNulo;
   VMref = pinv(J)*(hdp + W*tanh(0.9*he)) + EspNulo;
   

end