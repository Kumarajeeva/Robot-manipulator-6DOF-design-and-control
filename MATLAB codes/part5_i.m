%TRAJECTORIES OF A ROBOT MANIPULATOR

%MATLAB program to compute a CPS trajectory that passes through
the given points:

clc
clear all
close all
syms t
Theta=[0,-8,-90;0,45,90;0,30,90];%given theta values
Thetadot=[0,10,0;0,40,0;0,20,0];%given thetadot values
T=[0,2,4];%given time
Th_eq=[];
Thd_eq=[];
Thdd_eq=[];

%calculation
for i=1:2
th_i=Theta(:,i);
th_idot=Thetadot(:,i);
th_f=Theta(:,i+1);
th_fdot=Thetadot(:,i+1);
T_0=T(:,i);T_f=T(:,i+1);
A0=th_i; %calculating a0
A1=th_idot; %calculating a1
A2=(3*(th_f-th_i)-(2*th_idot+th_fdot)*(T_f-T_0))/(T_f-T_0)^2; %calculating a2
A3=(2*(th_i-th_f)+(th_idot+th_fdot)*(T_f-T_0))/(T_f-T_0)^3; %%calculating a3
th_eq=A0+A1*(t-T_0)+A2*(t-T_0)^2+A3*(t-T_0)^3; %calculating 𝜃(𝑡)
Th_eq=[Th_eq,th_eq];
thdot_eq=A1+2*A2*(t-T_0)+3*A3*(t-T_0)^2; %calculating 𝜃̇(𝑡)
Thd_eq=[Thd_eq,thdot_eq];
thddot_eq=2*A2+6*A3*(t-T_0); %calculating 𝜃̈(𝑡)
Thdd_eq=[Thdd_eq,thddot_eq];
end

% plotting CPS
for j=1:3
q1=0:0.2:2
Jx=vpa(subs(Th_eq(j,1),{t},{q1}));%substituting time in 𝜃(𝑡) for 0 to 2s
Jx1=vpa(subs(Thd_eq(j,1),{t},{q1})); %substituting time in 𝜃̇(𝑡) for 0 to 2s
Jx2=vpa(subs(Thdd_eq(j,1),{t},{q1})); %substituting time in 𝜃̈(𝑡) for 0 to 2s
q2=2:0.2:4
Nx_1=vpa(subs(Th_eq(j,2),{t},{q2})); }));%substituting time in 𝜃(𝑡) for 2 to 4s
Nx1_2=vpa(subs(Thd_eq(j,2),{t},{q2}));%substituting time in 𝜃̇(𝑡) for 2 to 4s
Nx2_2=vpa(subs(Thdd_eq(j,2),{t},{q2}));%substituting time in 𝜃̈(𝑡) for 2 to 4s
figure(j)
a=plot(q1,Jx,'-g');
hold on
b=plot(q2,Nx_1,'-g');
c=plot(q1,Jx1,'-r');
d=plot(q2,Nx1_2,'-r');
Jxx=[Jx2,Nx2_2];
Nxx=[q1,q2];
e=plot(Nxx,Jxx,'-b');
title( {'Trajectory for theta ';num2str(j)});
legend([a,c,e],"Position_","Velocity","Acceleration")
hold off
end