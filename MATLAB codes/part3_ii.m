%MATLAB program to obtain the Cartesian linear velocity of the origin of frame {4} of the MICROBOT
manipulator used in Part 2 when 𝜃1"= 35°/s, 𝜃2"= 20°/s and 𝜃3"= 45°/s at θ1= –18.43°, θ2= –29.18°, θ3= –111.38°.

clc
clear all
close all

%D-H parameters
a0 = 0;a1 = 0;a2 = 3;a3 = 4;
d1 = 2;d2 = 0;d3 = 0;d4 = 0;
alph0 = 0;alph1 = pi/2;alph2 = 0;alph3 = 0;
th1 = deg2rad(-18.43);th2 = deg2rad(-29.18);th3 = deg2rad(-111.38);th4 = deg2rad(0);
L1 = 0;L2 = 3;L3 = 4;

%Theta dots
td1 = deg2rad(35);
td2 = deg2rad(20);
td3 = deg2rad(45);
td4 = 0;

%Transformation matrix
T0_1 = [cos(th1) -sin(th1) 0 a0; sin(th1)*cos(alph0) cos(th1)*cos(alph0) -sin(alph0) -sin(alph0)*d1; sin(th1)*sin(alph0) cos(th1)*sin(alph0) cos(alph0) cos(alph0)*d1;0 0 0 1];
T1_2 = [cos(th2) -sin(th2) 0 a1; sin(th2)*cos(alph1) cos(th2)*cos(alph1) -sin(alph1) -sin(alph1)*d2; sin(th2)*sin(alph1) cos(th2)*sin(alph1) cos(alph1) cos(alph1)*d2;0 0 0 1];
T2_3 = [cos(th3) -sin(th3) 0 a2; sin(th3)*cos(alph2) cos(th3)*cos(alph2) -sin(alph2) -sin(alph2)*d3; sin(th3)*sin(alph2) cos(th3)*sin(alph2) cos(alph2) cos(alph2)*d3;0 0 0 1];
T3_4 = [cos(th4) -sin(th4) 0 a3; sin(th4)*cos(alph3) cos(th4)*cos(alph3) -sin(alph3) -sin(alph3)*d4; sin(th4)*sin(alph3) cos(th4)*sin(alph3) cos(alph3) cos(alph3)*d4;0 0 0 1];
Tb_w = T0_1*T1_2*T2_3*T3_4; %Final transformation matrix

%Position
P0_1 = [0;0;2];
P1_2 = [0;0;0];
P2_3 = [3;0;0];
P3_4 = [4;0;0];

%Transpose of rotation matrix
R1_0 = transpose(T0_1(1:3,1:3));
R2_1 = transpose(T1_2(1:3,1:3));
R3_2 = transpose(T2_3(1:3,1:3));
R4_3 = transpose(T3_4(1:3,1:3));

%Angular velocity
W0_0 = zeros(3,1);
W1_1 = R1_0*W0_0 + [0;0;td1];
W2_2 = R2_1*W1_1 + [0;0;td2];
W3_3 = R3_2*W2_2 + [0;0;td3];
W4_4 = R4_3*W3_3 + [0;0;td4];

%Linear velocity
V0_0 = zeros(3,1);
V1_1 = R1_0 * (V0_0 + cross(W0_0,P0_1));
V2_2 = R2_1 * (V1_1 + cross(W1_1,P1_2));
V3_3 = R3_2 * (V2_2 + cross(W2_2,P2_3));
V4_4 = R4_3 * (V3_3 + cross(W3_3,P3_4));

Cartesian_Velocity = [V4_4;W4_4] %Required Cartesian velocity