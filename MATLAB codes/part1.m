%WRIST TO FRAME HOMOGENEOUS TRANSFORMATION

%MATLAB program which computes the homogeneous transformation that relates the wrist frame to the
base frame, given the Denavit-Hartenberg parameters for an N degrees-of-freedom manipulator.

clear all
close all
clc
prompt='Enter values of i:'
n=input(prompt)

% Getting D-H parameters from the user
prompt='Enter values of alphaminusone:'
alpha_iminusone=input(prompt)
prompt='Enter the values of aminusone:'
a_iminusone=input(prompt)
prompt='Enter the values of d:'
d=input(prompt)
e=3;f=4;h=2;
theta=[-86.41 22.886 -3.4150 0]
F=1;

for i=1:n
A=[cosd(theta(i)) -sind(theta(i)) 0 a_iminusone(i)];
B=[(sind(theta(i)))*(cosd(alpha_iminusone(i))) (cosd(theta(i)))*(cosd(alpha_iminusone(i))) (-sind(alpha_iminusone(i))) (-sind(alpha_iminusone(i)))*(d(i))];
C=[(sind(theta(i)))*(sind(alpha_iminusone(i))) (cosd(theta(i)))*(sind(alpha_iminusone(i))) (cosd(alpha_iminusone(i))) (cosd(alpha_iminusone(i)))*(d(i))];
D=[0 0 0 1];
T=[A;B;C;D];
F=F*T %Transformation matrix
end