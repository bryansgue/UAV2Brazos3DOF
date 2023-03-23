clc, clear all, close all
%load('Cinematica.mat')
xu = 5; 
yu = 5; 
zu = 5;
psi= -50* (pi/180);

%Priemra Brazo
q1_1 = 30 * (pi/180);
q2_1 = 80 * (pi/180);
q3_1 = -25 * (pi/180);
l1_1 = 0.15; l2_1 = 0.44; l3_1 = 0.45;

% Segundo brazo
q1_2 = -30 * (pi/180);
q2_2 = 50 * (pi/180);
q3_2 = -80 * (pi/180);
l1_2 = 0.15; l2_2 = 0.44; l3_2 = 0.45;

% Desplazamientos
a_1=0; 
b_1=0.15; 
c_1=-0.03;

a_2=0; 
b_2=-0.15; 
c_2=-0.03;

h_1 = CDBrazo3DOF(xu,yu,zu,psi,l1_1,l2_1,l3_1,a_1,b_1,c_1,q1_1,q2_1,q3_1);
h_2 = CDBrazo3DOF(xu,yu,zu,psi,l1_2,l2_2,l3_2,a_2,b_2,c_2,q1_2,q2_2,q3_2);

figure(1)
  axis equal
%   view(0,90) % Angulo de vista
  cameratoolbar
  title ("Resultados Experimentales")
%%
tic
DimensionesManipulador(0,l1_1,l2_1,l3_1-0.2,1); 
Hexacoptero(0.05,[1 0 0]);
% a) GrÃ¡fica inicial del brazo
  M_1=Manipulador3D(xu,yu,zu,psi,a_1,b_1,c_1,q1_1,q2_1,q3_1,0); % GrÃ¡fica el brazo por encima el UAV
  rotate(M_1,[1 0 0],180,[xu,yu,zu])
  hold on
  M_2=Manipulador3D(xu,yu,zu,psi,a_2,b_2,c_2,q1_2,q2_2,q3_2,0); % GrÃ¡fica el brazo por encima el UAV
  rotate(M_2,[1 0 0],180,[xu,yu,zu])
  hold on
% 2) Configura escala y color del UAV

 UAV= Hexacoptero(xu,yu,zu,psi);
 toc
%%
%   plot3(x,y,z,'o');
  plot3(h_1(1),h_1(2),h_1(3),'*');
  plot3(h_2(1),h_2(2),h_2(3),'*');
 