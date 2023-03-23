%% *************************************
%% CONTROL CINEMATICO UAV CON DOS BRAZOS
%% *************************************
%% Inicializacion
tic
clc; clear all; close all; warning off % Inicializacion
ts = 0.1;       
tfin = 160;     
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
q2_1(1) = 60 * (pi/180);
q3_1(1) = -30 * (pi/180);
% Segundo brazo
q1_2(1) = 0 * (pi/180);
q2_2(1) = 60 * (pi/180);
q3_2(1) = -30 * (pi/180);
%% Tarea deseada
% Silla de montar
xd_0 = 1.9*cos(0.05*t)+1.75;      xdp_0 = -1.9*0.05*sin(0.05*t);  xdpp_0 = -1.9*0.05*0.05*cos(0.05*t);    
yd_0 = 1.9*sin(0.05*t)+1.75;      ydp_0 =  1.9*0.05*cos(0.05*t);  ydpp_0 = -1.9*0.05*0.05*sin(0.05*t); 
zd_0 = 0.2*sin(0.2*t)+2;          zdp_0 =  0.2*0.2*cos(0.2*t); 
% Ocho
xd_0 = 5 * sin(0.04*t)+1.75;       xdp_0 = 5*0.04*cos(0.04*t);     xdpp_0 = -5*0.04*0.04*sin(0.04*t);
yd_0 = 5 * sin(0.08*t)+1.75;       ydp_0 = 5*0.08*cos(0.08*t);     ydpp_0 = -5*0.08*0.08*sin(0.08*t);               
zd_0 = 1 * sin (0.08 * t) +2 ;    zdp_0 = 0.08*cos(0.08*t);
% Orientacion deseada del UAV
psid= (atan2(ydp_0,xdp_0));
psidp = (1./((ydp_0./xdp_0).^2+1)).*((ydpp_0.*xdp_0-ydp_0.*xdpp_0)./xdp_0.^2);
psidp(1)=0;
% Distancia deseada entre los brazos
d= 0.7+0.3*cos(0.5*t);
% Angulo theta deseado
thetad = psid + pi/2;
% Angulo phi deseado
phid = pi/2 + (45*cos(0.1*t))*pi/180;

xd_1 = xd_0 + (d/2).*sin(phid).*cos(thetad); xdp_1 = diff(xd_1)/ts;
yd_1 = yd_0 + (d/2).*sin(phid).*sin(thetad); ydp_1 = diff(yd_1)/ts;
zd_1 = zd_0 + (d/2).*cos(phid);              zdp_1 = diff(zd_1)/ts; 

xd_2 = xd_0 - (d/2).*sin(phid).*cos(thetad); xdp_2 = diff(xd_2)/ts;
yd_2 = yd_0 - (d/2).*sin(phid).*sin(thetad); ydp_2 = diff(yd_2)/ts;
zd_2 = zd_0 - (d/2).*cos(phid);              zdp_2 = diff(zd_2)/ts;

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

for k=1:length(t)-1
    tic
%% 1) LEY DE CONTROL DEL MANIPULADOR AEREO
    u = Control_Jacob(l2_1,l3_1,l2_2,l3_2,a_1,b_1,a_2,b_2,q1_1(k),q2_1(k),q3_1(k),q1_2(k),q2_2(k),q3_2(k),psi(k),hx_1(k),hy_1(k),hz_1(k),hx_2(k),hy_2(k),hz_2(k),xd_1(k),yd_1(k),zd_1(k),xd_2(k),yd_2(k),zd_2(k),psid(k),xdp_1(k),ydp_1(k),zdp_1(k),xdp_2(k),ydp_2(k),zdp_2(k),psidp(k)) ;

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
    q3_2(k+1)= q3_2(k) + qp3_2(k)*ts;3.33
    
%% 5) Posicion de los extremo operativos
    h_1 = CDBrazo3DOF(xu(k+1),yu(k+1),zu(k+1),psi(k+1),l1_1,l2_1,l3_1,a_1,b_1,c_1,q1_1(k+1),q2_1(k+1),q3_1(k+1));
    hx_1(k+1) = h_1(1);
    hy_1(k+1) = h_1(2);
    hz_1(k+1) = h_1(3);
    h_2 = CDBrazo3DOF(xu(k+1),yu(k+1),zu(k+1),psi(k+1),l1_2,l2_2,l3_2,a_2,b_2,c_2,q1_2(k+1),q2_2(k+1),q3_2(k+1));
    hx_2(k+1) = h_2(1);
    hy_2(k+1) = h_2(2);
    hz_2(k+1) = h_2(3);
    
 %% 6) Posicion del punto central
    hx_0(k+1) = (hx_1(k+1)+hx_2(k+1))/2;
    hy_0(k+1) = (hy_1(k+1)+hy_2(k+1))/2;
    hz_0(k+1) = (hz_1(k+1)+hz_2(k+1))/2;   
    
   
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
    plot3(xd_1,yd_1,zd_1,'Color','red','LineStyle','--','linewidth',1.25)
    plot3(xd_2,yd_2,zd_2,'Color','blue','LineStyle','--','linewidth',1.25)
    plot3(xd_0,yd_0,zd_0,'Color','green','LineStyle','--','linewidth',1.25)
    H1 = plot3(hx_1(1),hy_1(1),hz_1(1),':r');
    H2 = plot3(hx_2(1),hy_2(1),hz_2(1),':b');
    H3 = line([hx_1(1) hx_2(1)] ,[hy_1(1)  hy_2(1)],[hz_1(1) hz_2(1)]);
    H4 = plot3(hx_0(1),hy_0(1),hz_0(1),'o');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

%     vid = VideoWriter('tutorial');
%     open(vid);


disp('Empieza el programa')

%axis tight manual 
%set(gca,'nextplot','replacechildren'); 
%v = VideoWriter('peaks.avi');
%open(v);
%% Animacion de movimiento
  for k=1:50:length(t)  
    tic
    drawnow
%     T(k) = getframe();
    delete(M_1);
    delete(M_2);
    delete(UAV);
    delete(H1);
    delete(H2);
    delete(H3);
    delete(H4);
    
    H1 = plot3(hx_1(1:k),hy_1(1:k),hz_1(1:k),'r');
    H2 = plot3(hx_2(1:k),hy_2(1:k),hz_2(1:k),'b');
    H5 = plot3(hx_0(2:k),hy_0(2:k),hz_0(2:k),'--g');
    H3 = line([hx_1(k) hx_2(k)] ,[hy_1(k)  hy_2(k)],[hz_1(k) hz_2(k)],'Color','red','LineStyle','-'); hold on
    H4 = plot3(hx_0(k),hy_0(k),hz_0(k),'o','Color','red');
        
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
    
%     frame = getframe(gcf);
%     writeVideo(v,frame);
    toc
    pause(0.0)
  end
%%
%close(v);
% writeVideo(vid,T);
% close(vid);
%%
%******************************************************************************************************************
%********************************************* GRÃ?FICAS ***********************************************************
%% ****************************************************************************************************************

% 2) CÃ¡lculos del Error
  figure(2)
  hxe= xd_0 - hx_0;
  hye= yd_0 - hy_0;
  hze= zd_0 - hz_0;
  psie= psid-psi;
  plot(hxe); hold on
  plot(hye); hold on
  plot(hze);hold on
  plot(psie);
  legend("hxe","hye","hze","psie")
  title ("Errores de posicion")
  
  % 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
  figure(3)
  
  subplot(3,1,1)
  plot(ul); hold on
  plot(um); hold on
  plot(un); hold on
  plot(w); hold on
  legend("ul","um","un","w")
  ylabel('velocidad lineal [m/s]'); xlabel('s [ms]');
  title ("Velocidad del Manipulador aereo dual")
  
  subplot(3,1,2)
  plot(qp1_1); hold on
  plot(qp2_1); hold on
  plot(qp3_1); hold on
  legend("qp1_1","qp1_1","qp2_1")
  ylabel('velocidad angular [rad/s]'); xlabel('s [ms]');

  subplot(3,1,3)
  plot(qp1_2); hold on
  plot(qp2_2); hold on
  plot(qp3_2); hold on
  legend("qp1_2","qp2_2","qp3_2")
  ylabel('velocidad angular [rad/s]'); xlabel('s [ms]');

  
