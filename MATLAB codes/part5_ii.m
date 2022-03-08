%TRAJECTORIES OF A ROBOT MANIPULATOR

%Computing the LSPB trajectories that passes through the same points defined in part5_i and tb = 0.5s for all the
segments.

clc
clear all
close all

syms t;
Theta=[0,-8,-90;0,45,90;0,30,90];%given theta values
Thetadot=[0,10,0;0,40,0;0,20,0];%given thetadot values
T=[0,2,4]; %given time
Trajectory_1=[]; %null matrix
Trajectory_2=[];
Trajectory_3=[];
t_b=0.5;
Velocity_ans=[];
for i=1:2
th_i=Theta(:,i);
th_idot=Thetadot(:,i);
th_f=Theta(:,i+1);
th_fdot=Thetadot(:,i+1);
T_0=T(:,i);
T_f=T(:,i+1);
Velocity=(th_f-th_i-0.5*t_b*(th_idot+th_fdot))/(T_f-(t_b+T_0)); %calculating velocity
Velocity_ans=[Velocity_ans Velocity];

%calculating 𝜃(𝑡) for 0≤𝑡≤𝑡𝑏, 𝑡𝑏≤𝑡≤𝑡𝑓−𝑡𝑏, 𝑡𝑓−𝑡𝑏≤𝑡≤𝑡𝑓
traj_1=th_i+th_idot*(t-T_0)+0.5*(Velocity-th_idot)*(t-T_0)^2/t_b; % 0≤𝑡≤𝑡𝑏
Trajectory_1=[Trajectory_1,traj_1];
traj_2=Velocity*(t-T_0)+(th_i+0.5*th_idot*t_b-0.5*Velocity*t_b); %𝑡𝑏≤𝑡≤𝑡𝑓−𝑡𝑏
Trajectory_2=[Trajectory_2,traj_2];
traj_3=(th_f-th_fdot*T_f+((th_fdot-Velocity)*T_f^2)/(2*t_b))+(th_fdot-(th_fdot-Velocity)*T_f/t_b)*(t)+((th_fdot-Velocity)*((t)^2)/(2*t_b)); %𝑡𝑓−𝑡𝑏≤𝑡≤𝑡𝑓
Trajectory_3=[Trajectory_3,traj_3];
end

%Calculating 𝜃̇ and 𝜃̈
Trajdot1=diff(Trajectory_1,t);
Trajdot2=diff(Trajectory_2,t);
Trajdot3=diff(Trajectory_3,t);
Trajddot1=diff(Trajdot1,t);
Trajddot2=diff(Trajdot2,t);
Trajddot3=diff(Trajdot3,t);
%Plotting LSPB
b=4;

%For three segments
for j=1:3
T_0=0;T_f=2;
q1=T_0:0.1:t_b+T_0;

%substituting time
Trx_1=vpa(subs(Trajectory_1(j,1),{t},{q1}));
Trx_d1=vpa(subs(Trajdot1(j,1),{t},{q1}));
Trx_dd1=vpa(subs(Trajddot1(j,1),{t},{q1}));
T_0=T_0+2;T_f=T_f+2;
k11=T_0:0.1:t_b+T_0;

Trxx_1=vpa(subs(Trajectory_1(j,2),{t},{k11}));
Trxx_d1=vpa(subs(Trajdot1(j,2),{t},{k11}));
Trxx_dd1=vpa(subs(Trajddot1(j,2),{t},{k11}));
T_0=0;T_f=2;
q2=t_b+T_0:0.1:T_f-t_b;

Trx_2=vpa(subs(Trajectory_2(j,1),{t},{q2}));
Trx_d2=vpa(subs(Trajdot2(j,1),{t},{q2}));
Trx_dd2=vpa(subs(Trajddot2(j,1),{t},{q2}));
T_0=T_0+2;T_f=T_f+2;
k21=t_b+T_0:0.1:T_f-t_b;

Trxx_2=vpa(subs(Trajectory_2(j,2),{t},{k21}));
Trxx_d2=vpa(subs(Trajdot2(j,2),{t},{k21}));
Trxx_dd2=vpa(subs(Trajddot2(j,2),{t},{k21}));
T_0=0;T_f=2;
k3=T_f-t_b:0.1:T_f;

Trx_3=vpa(subs(Trajectory_3(j,1),{t},{k3}));
Trx_d3=vpa(subs(Trajdot3(j,1),{t},{k3}));
Trx_dd3=vpa(subs(Trajddot3(j,1),{t},{k3}));
T_0=T_0+2;T_f=T_f+2;

k31=T_f-t_b:0.1:T_f;
Trx_3x=vpa(subs(Trajectory_3(j,2),{t},{k31}));
Trxx_d3=vpa(subs(Trajdot3(j,2),{t},{k31}));
Trxx_dd3=vpa(subs(Trajddot3(j,2),{t},{k31}));

figure(b)
subplot(2,2,1);
plot(q1,Trx_1,k11,Trxx_1);
hold on
plot(q2,Trx_2,k21,Trxx_2);
plot(k3,Trx_3,k31,Trx_3x);
title( {'Position for theta ';num2str(j)});
hold off

subplot(2,2,2);
plot(q1,Trx_d1,k11,Trxx_d1);
hold on
plot(q2,Trx_d2,k21,Trxx_d2);
plot(k3,Trx_d3,k31,Trxx_d3);
title( {'Velocity for theta ';num2str(j)});
hold off

subplot(2,2,3);
ac_p=[Trx_dd1,Trx_dd2,Trx_dd3,Trxx_dd1,Trxx_dd2,Trxx_dd3];
ac_pt=[q1,q2,k3,k11,k21,k31];
plot(ac_pt,ac_p);
title( {'Acceleration for theta ';num2str(j)});
%hold off
b=b+1;
end