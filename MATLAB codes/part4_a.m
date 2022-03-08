%INVERSE DYNAMICS OF A ROBOT MANIPULATOR USING NETWON-EULER ALGORITHM

%MATLAB program to obtain the Cartesian linear velocity of the origin of frame {4} of the MICROBOT
manipulator used in Part 2 when 𝜃1(t)= 𝜃2(t)= 𝜃3(t)= 45[1 + 6e^-t/0.3 – 8e^-t/0.4] for 0≤t≤2 s. Assume that all the mass of
each link can be considered as a point mass located at the distal end of the link. Plot the joint positions, velocities,
accelerations and torques against time

clc
clear all
close all
syms t

%D-H parametes
a0 = 0;a1 = 0;a2 = 3;a3 = 4;
d1 = 2;d2 = 0;d3 = 0;d4 = 0;
alpha0 = 0;alpha1 = pi/2;alpha2 = 0;alpha3 = 0;
L1 = 0;L2 = 3;L3 = 4;

%given position
th1 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
th2 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
th3 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
th4 = deg2rad(0);

%calculating velocity from given position
td1 = diff(th1);
td2 = diff(th2);
td3 = diff(th3);
td4 = 0;

%calculating acceleration from given position
tdd1 = diff(td1);
tdd2 = diff(td3);
tdd3 = diff(td3);
tdd4 = 0;
M1 = 4;M2 = 2;M3 = 2; %masses
g = 9.8; %gravitational constant
T0_1 = [cos(th1) -sin(th1) 0 a0; sin(th1)*cos(alpha0) cos(th1)*cos(alpha0) -sin(alpha0) -sin(alpha0)*d1; sin(th1)*sin(alpha0) cos(th1)*sin(alpha0) cos(alpha0) cos(alpha0)*d1;0 0 0 1];
T1_2 = [cos(th2) -sin(th2) 0 a1; sin(th2)*cos(alpha1) cos(th2)*cos(alpha1) -sin(alpha1) -sin(alpha1)*d2; sin(th2)*sin(alpha1) cos(th2)*sin(alpha1) cos(alpha1) cos(alpha1)*d2;0 0 0 1];
T2_3 = [cos(th3) -sin(th3) 0 a2; sin(th3)*cos(alpha2) cos(th3)*cos(alpha2) -sin(alpha2) -sin(alpha2)*d3; sin(th3)*sin(alpha2) cos(th3)*sin(alpha2) cos(alpha2) cos(alpha2)*d3;0 0 0 1];
T3_4 = [cos(th4) -sin(th4) 0 a3; sin(th4)*cos(alpha3) cos(th4)*cos(alpha3) -sin(alpha3) -sin(alpha3)*d4; sin(th4)*sin(alpha3) cos(th4)*sin(alpha3) cos(alpha3) cos(alpha3)*d4;0 0 0 1];
Tb_w = T0_1*T1_2*T2_3*T3_4; %Homogenous transformation matrix

%for inward iterations
R1_0 = transpose(T0_1(1:3,1:3));
R2_1 = transpose(T1_2(1:3,1:3));
R3_2 = transpose(T2_3(1:3,1:3));
R4_3 = transpose(T3_4(1:3,1:3));

%for outward iterations
r0_1 = T0_1(1:3,1:3);
r1_2 = T1_2(1:3,1:3);
r2_3 = T2_3(1:3,1:3);
r3_4 = T3_4(1:3,1:3);

%Positions from the T matrix and centre of mass
p0_1 = zeros(3,1);
p1_2 = [L1;0;0];
p2_3 = [L2;0;0];
p3_4 = [L3;0;0];
pc1_1 = [L1;0;0];
pc2_2 = [L2;0;0];
pc3_3 = [L3;0;0];

%Assumptions for inertia
IC11 = zeros(3,3);
IC22 = zeros(3,3);
IC33 = zeros(3,3);

%Initial assumptions
VD0_0 = [0;g;0];
f4_4 = zeros(3,1);
n4_4 = zeros(3,1);
w0_0 = zeros(3,1);
wd0_0 = zeros(3,1);


%OUTWARD RECURSIONS

%link 1
w1_1 = R1_0*w0_0 + [0;0;td1]; %angular velocity of link1 w.r.t frame1
wd1_1 = R1_0*wd0_0 + cross(R1_0*w0_0,[0;0;td1]) + [0;0;tdd1]; %angular acceleration of link1 w.r.t frame1
vd1_1 = R1_0*(cross(wd0_0,p0_1) + cross(w0_0 , cross(w0_0,p0_1)) + VD0_0); %linear acceleration of link1 w.r.t frame1
vdc1_1 = cross(wd1_1,pc1_1) + cross(w1_1, cross(w1_1,pc1_1)) + vd1_1; %linear acceleration of centre of mass of link1
F1_1 = M1*vdc1_1; %Force acting on link1
N1_1 = IC11*wd1_1 + cross(w1_1,IC11*w1_1); %Moment acting on link1

%link 2
w2_2 = R2_1*w1_1 + [0;0;td2];
wd2_2 = R2_1*wd1_1 + cross(R2_1*w1_1,[0;0;td2]) + [0;0;tdd2];
vd2_2 = R2_1*(cross(wd1_1,p1_2) + cross(w1_1 , cross(w1_1,p1_2)) + vd1_1);
vdc2_2 = cross(wd2_2,pc2_2) + cross(w2_2, cross(w2_2,pc2_2)) + vd2_2;
F2_2 = M2*vdc2_2;
N2_2 = IC22*wd2_2 + cross(w1_1,IC11*w1_1);

%link 3
w3_3 = R3_2*w2_2 + [0;0;td3];
wd3_3 = R3_2*wd2_2 + cross(R3_2*w2_2,[0;0;td3]) + [0;0;tdd3];
vd3_3 = R3_2*(cross(wd2_2,p2_3) + cross(w2_2 , cross(w2_2,p2_3)) + vd2_2);
vdc3_3 = cross(wd3_3,pc3_3) + cross(w3_3, cross(w3_3,pc3_3)) + vd3_3;
F3_3 = M3*vdc3_3;
N3_3 = IC33*wd3_3 + cross(w2_2,IC22*w2_2);


%INWARD RECURSIONS

%link 3
f3_3 = r3_4*f4_4 + F3_3;
n3_3 = N3_3 + r3_4*n4_4 + cross(pc3_3,F3_3) + cross(p3_4,r3_4*f4_4);

%link 2
f2_2 = r2_3*f3_3 + F2_2;
n2_2 = N2_2 + r2_3*n3_3 + cross(pc2_2,F2_2) + cross(p2_3,r2_3*f3_3);

%link 1
f1_1 = r1_2*f2_2 + F1_1;
n1_1 = N1_1 + r1_2*n2_2 + cross(pc1_1,F1_1) + cross(p1_2,r1_2*f2_2);

%Joint torques
time = 0:0.05:2;
Torque1 = transpose(n1_1)*[0;0;1]
Torque1_plot=vpa(subs(Torque1,t,time)); %Torque of joint1
Torque2 = transpose(n2_2)*[0;0;1]
Torque2_plot=vpa(subs(Torque2,t,time)); %Torque of joint2
Torque3 = transpose(n3_3)*[0;0;1]
Torque3_plot=vpa(subs(Torque3,t,time)); %Torque of joint3

%Plotting torques of the joints against time
figure(1)
subplot(2,2,1)
plot(time,Torque1_plot,'b')
title('Torque for joint1')
subplot(2,2,2)
plot(time,Torque2_plot,'g')
title('Torque for joint2')
subplot(2,2,3)
plot(time,Torque3_plot,'r')
title('Torque for joint3')
hold off

%Plotting position
th1_plot=vpa(subs(th1,t,time));
figure(2)
plot(time,th1_plot,’g’)
title('Position for all joints')

%Plotting velocity
td1_plot=vpa(subs(td1,t,time));
figure(3)
plot(time,td1_plot,’r’)
title('Velocity for all joints')

%Plotting acceleration
tdd1_plot=vpa(subs(tdd1,t,time));
figure(4)
plot(time,tdd1_plot)
title('acceleration for all joints')