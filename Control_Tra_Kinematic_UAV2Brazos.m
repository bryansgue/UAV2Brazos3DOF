% ******************************************************************************************************************
% ************************************ SEGUIMIENTO DE TRAYECTORIA **************************************************
% ********************************** ROBOT MANIPULADOR AEREO DOS BRAZOS *****************************************************
% ******************************************************************************************************************
%% Inicializacion
tic
clc; clear all; close all; warning off % Inicializacion
ts = 0.1;       
tfin = 200;     
t = 0:ts:tfin;
%% Posicion de los brazos roboticos
a_1=0; 
b_1=0.18; 
c_1=-0.03;
a_2=0; 
b_2=-0.15; 
c_2=-0.03;
%% Eslabones del brazo
l1_1 = 0.15; 
l2_1 = 0.44; 
l3_1 = 0.45;
l1_2 = 0.15; 
l2_2 = 0.44; 
l3_2 = 0.45;
%% Condiciones iniciales del robot
% UAV
xu(1) = 0; 
yu(1) = 0; 
zu(1) = 5;
psi(1)= 0 * (pi/180);
% Primer brazo
q1_1(1) = 0 * (pi/180);
q2_1(1) = 0 * (pi/180);
q3_1(1) = 0 * (pi/180);
% Segundo brazo
q1_2(1) = 0 * (pi/180);
q2_2(1) = 0 * (pi/180);
q3_2(1) = 0 * (pi/180);
%% Tarea deseada
%Silla de montar  
%%Extremo Operativo 1
xd_2 = 1.9*cos(0.05*t)+1.75;      xdp_2 = -1.9*0.05*sin(0.05*t);      
yd_2 = 1.9*sin(0.05*t)+1.75;      ydp_2 =  1.9*0.05*cos(0.05*t);     
zd_2 = 0.2*sin(0.2*t)+2;          zdp_2 =  0.2*0.2*cos(0.2*t); 
    
%%Extremo Operativo 2
xd_1 = 1.6*cos(0.05*t)+1.75;      xdp_1 = -1.6*0.05*sin(0.05*t);      xdpp_1 = -1.6*0.05*0.05*cos(0.05*t);
yd_1 = 1.6*sin(0.05*t)+1.75;      ydp_1 =  1.6*0.05*cos(0.05*t);      ydpp_1 = -1.6*0.05*0.05*sin(0.05*t);
zd_1 = 0.2*sin(0.3*t)+2.3;        zdp_1 =  0.2*0.3*cos(0.3*t); 
%UAV
psid= (atan2(ydp_1,xdp_1));
psidp = (1./((ydp_1./xdp_1).^2+1)).*((ydpp_1.*xdp_1-ydp_1.*xdpp_1)./xdp_1.^2);
psidp(1)=0;

hdp = [xdp_1 ydp_1 zdp_1 xdp_2 ydp_2 zdp_2 psidp]';
%% Cinematica directa inicial de los extremos operativos
h = CDBrazo3DOF(xu(1),yu(1),zu(1),psi(1),l1_1,l2_1,l3_1,a_1,b_1,c_1,q1_1(1),q2_1(1),q3_1(1));
hx_1(1) = h(1);
hy_1(1) = h(2);
hz_1(1) = h(3);
h = CDBrazo3DOF(xu(1),yu(1),zu(1),psi(1),l1_2,l2_2,l3_2,a_2,b_2,c_2,q1_2(1),q2_2(1),q3_2(1));
hx_2(1) = h(1);
hy_2(1) = h(2);
hz_2(1) = h(3);
%******************************************************************************************************************
%***************************************** CONTROLADOR ***********************************************************
%*****************************************************************************************************************
disp('Empieza el programa')

for k=1:length(t)
    tic
%% 1) LEY DE CONTROL DEL MANIPULADOR AEREO
    u = Control_Jacob(l2_1,l3_1,l2_2,l3_2,a_1,b_1,a_2,b_2,q1_1(k),q2_1(k),q3_1(k),q1_2(k),q2_2(k),q3_2(k),psi(k),hx_1(k),hy_1(k),hz_1(k),hx_2(k),hy_2(k),hz_2(k),xd_1(k),yd_1(k),zd_1(k),xd_2(k),yd_2(k),zd_2(k),psid(k),xdp_1(k),ydp_1(k),zdp_1(k),xdp_2(k),ydp_2(k),zdp_2(k),psidp(k));

    ul(k)  = u(1);
    um(k)  = u(2);
    un(k)  = u(3);
    w(k)   = u(4);
    qp1_1(k)= u(5);
    qp2_1(k)= u(6);
    qp3_1(k)= u(7);
    qp1_2(k)= u(8);
    qp2_2(k)= u(9);
    qp3_2(k)= u(10);
       
%% 2) CINEMATICA DEL UAV
    xu_p(k) = ul(k) * cos(psi(k)) - um(k) * sin(psi(k));
    yu_p(k) = ul(k) * sin(psi(k)) + um(k) * cos(psi(k));
    zu_p(k) = un(k);
  
%% 3) Posicion del UAV en k+1 (Integracion Euler)
    xu(k+1) = xu_p(k)*ts + xu(k);
    yu(k+1) = yu_p(k)*ts + yu(k);
    zu(k+1) = zu_p(k)*ts + zu(k);      
    psi(k+1) = Angulo(w(k)*ts + psi(k));

%% 4) Posicion de las articulaciones en k+1 (Integracion Euler)
    % Brazo 1
    q1_1(k+1)= q1_1(k) + qp1_1(k)*ts;
    q2_1(k+1)= q2_1(k) + qp2_1(k)*ts;
    q3_1(k+1)= q3_1(k) + qp3_1(k)*ts;
    % Brazo 2
    q1_2(k+1)= q1_2(k) + qp1_2(k)*ts;
    q2_2(k+1)= q2_2(k) + qp2_2(k)*ts;
    q3_2(k+1)= q3_2(k) + qp3_2(k)*ts;
    
%% 5) Posicion de los extremo operativos
    h_1 = CDBrazo3DOF(xu(k+1),yu(k+1),zu(k+1),psi(k+1),l1_1,l2_1,l3_1,a_1,b_1,c_1,q1_1(k+1),q2_1(k+1),q3_1(k+1));
    hx_1(k+1) = h_1(1);
    hy_1(k+1) = h_1(2);
    hz_1(k+1) = h_1(3);
    h_2 = CDBrazo3DOF(xu(k+1),yu(k+1),zu(k+1),psi(k+1),l1_2,l2_2,l3_2,a_2,b_2,c_2,q1_2(k+1),q2_2(k+1),q3_2(k+1));
    hx_2(k+1) = h_2(1);
    hy_2(k+1) = h_2(2);
    hz_2(k+1) = h_2(3);
%% 6) Tiempo de Maquina 
     dt(k) = toc;
end
disp('Fin de los calculos')

%******************************************************************************************************************
%********************************* ANIMACION SEGUIMIENTO DE TRAYECTORIA ******************************************
%% ******************************************************************************************************************
  disp('Play Animacion de Simulacion ')
  figure(1)
  axis equal
  view(-15,15) % Angulo de vista
  cameratoolbar
  title ("Simulacion Cinematica")
%% Configuracion del Manipulador Aereo
  DimensionesManipulador(0,l1_1,l2_1,l3_1-0.2,1);
  Hexacoptero(0.05,[1 0 0]);
%% Dibujo del Manipulador Aereo
  %) a) Manipulador 1
  M_1=Manipulador3D(xu(1),yu(1),zu(1),psi(1),a_1,b_1,c_1,q1_1(1),q2_1(1),q3_1(1),0); % GrÃ¡fica el brazo por encima el UAV
  rotate(M_1,[1 0 0],180,[xu(1),yu(1),zu(1)])
  hold on
  % b) Manipulador 2
  M_2=Manipulador3D(xu(1),yu(1),zu(1),psi(1),a_2,b_2,c_2,q1_2(1),q2_2(1),q3_2(1),0); % GrÃ¡fica el brazo por encima el UAV
  rotate(M_2,[1 0 0],180,[xu(1),yu(1),zu(1)])
  hold on
  % c) UAV
  UAV= Hexacoptero(xu(1),yu(1),zu(1),psi(1));
%% Plot punto deseado
    plot3(xd_1,yd_1,zd_1)
    plot3(xd_2,yd_2,zd_2)
    H1 = plot3(hx_1(1),hy_1(1),hz_1(1),'r');
    H2 = plot3(hx_2(1),hy_2(1),hz_2(1),'b');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
  
%% Animacion de movimiento
  for k=1:50:length(t)  
    tic
    drawnow
    delete(M_1);
    delete(M_2);
    delete(UAV);
    delete(H1);
    delete(H2);
    
    H1 = plot3(hx_1(1:k),hy_1(1:k),hz_1(1:k),'r');
    H2 = plot3(hx_2(1:k),hy_2(1:k),hz_2(1:k),'b');
        
    % a) Manipulador 1
    M_1=Manipulador3D(xu(k),yu(k),zu(k),psi(k),a_1,b_1,c_1,q1_1(k),q2_1(k),q3_1(k),0); % GrÃ¡fica el brazo por encima el UAV
    rotate(M_1,[1 0 0],180,[xu(k),yu(k),zu(k)])
    hold on
    % b) Manipulador 2
    M_2=Manipulador3D(xu(k),yu(k),zu(k),psi(k),a_2,b_2,c_2,q1_2(k),q2_2(k),q3_2(k),0); % GrÃ¡fica el brazo por encima el UAV
    rotate(M_2,[1 0 0],180,[xu(k),yu(k),zu(k)])
    hold on
    % c) UAV
    UAV= Hexacoptero(xu(k),yu(k),zu(k),psi(k));
    toc
    pause(0.0)
  end
%%
%******************************************************************************************************************
%********************************************* GRÃ?FICAS ***********************************************************
%% ****************************************************************************************************************

% 1) Igualar columnas de los vectores creados
%   hx(:,end)=[];
%   hy(:,end)=[];
%   hz(:,end)=[];
%   psi(:,end)=[];
  %%
% 2) CÃ¡lculos del Error
  figure(2)
  hxe= xd_1 - hx_1(1:end-1);
  hye= yd_1 - hy_1(1:end-1);
  hze= zd_1 - hz_1(1:end-1);
  psie= Angulo(psid-psi(:,end-1));
  plot(hxe), hold on, grid on
  plot(hye)
  plot(hze)
%   plot(psie)
  legend("hxe","hye","hze","psie")
  title ("Errores de posiciÃ³n")
  %%
% % 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
%   figure(3)
%   
%   subplot(4,1,1)
%   plot(xd)
%   hold on
%   plot(hx)
%   legend("xd","hx")
%   ylabel('x [m]'); xlabel('s [ms]');
%   title ("Posiciones deseadas y reales del extremo operativo del manipulador aÃ©reo")
%   
%   subplot(4,1,2)
%   plot(yd)
%   hold on
%   plot(hy)
%   legend("yd","hy")
%   ylabel('y [m]'); xlabel('s [ms]');
% 
%   subplot(4,1,3)
%   plot(zd)
%   hold on
%   plot(hz)
%   grid on
%   legend("zd","hz")
%   ylabel('z [m]'); xlabel('s [ms]');
% 
%   subplot(4,1,4)
%   plot(Angulo(psid))
%   hold on
%   plot(psi)
%   legend("psid","psi")
%   ylabel('psi [rad]'); xlabel('s [ms]');