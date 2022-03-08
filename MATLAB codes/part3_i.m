%CARTESIAN VELOCITY OF FRAMES AND END EFFECTORS

%MATLAB program to compute the Cartesian velocity of the end-effector, given a set of joint velocities
and the manipulator’s configuration.

clear all
close all
clc
syms theta1 theta2 theta3 theta1dot theta2dot theta3dot L1 L2 ;

alpha_iminusone=[0 0 0];
a_iminusone=[0;L1;L2];
d=[0 0 0];
theta=[theta1 theta2 theta3];

T0to1=[cos(theta1) (-sin(theta1)) 0 0;sin(theta1) cos(theta1) 0 0;0 0 1 0;0 0 0 1]
R0to1=T0to1(1:3,1:3)
P0to1=T0to1(1:3,4)
T1to2=[cos(theta2) (-sin(theta2)) 0 L1;(sin(theta2)) (cos(theta2)) 0 0;0 0 1 0;0 0 0 1]
R1to2=T1to2(1:3,1:3)
P1to2=T1to2(1:3,4)
T2to3=[1 0 0 L2;0 1 0 0;0 0 1 0;0 0 0 1]
R2to3=T2to3(1:3,1:3)
P2to3=T2to3(1:3,4)
omega0wrt0=[0;0;0]

V0wrt0=[0;0;0]
omega1wrt1=(((R0to1))'*(omega0wrt0)+[0;0;theta1dot])
V1wrt1=((R0to1)*((V0wrt0)+(cross(omega0wrt0,P0to1))))
omega2wrt2=(((R1to2)')*(omega1wrt1)+[0;0;theta2dot])
V2wrt2=(((R1to2)')*((V1wrt1)+(cross(omega1wrt1,P1to2))))
omega3wrt3=(((R2to3)')*(omega2wrt2)+[0;0;theta3dot]) %Angular velocity
V3wrt3=(((R2to3)')*((V2wrt2)+(cross(omega2wrt2,P2to3)))) %Linear velocity

Cartesianvelocity=[V3wrt3;omega3wrt3]