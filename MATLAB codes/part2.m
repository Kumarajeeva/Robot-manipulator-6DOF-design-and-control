%INVERSE KINEMATICS OF THE MANIPULATOR

%MATLAB program to compute all possible solutions to the inverse kinematics problem for
the manipulator (for position only). Check the results of your program using the MATLAB program written in
Part 1.

clear all
close all
clc
Pos_x=3;Pos_y=-1;Pos_z=4.5;%given positions
h=2;e=3;f=4;


% theta1 anticlockwise and elbow up
th_1=atan2d(Pos_y,Pos_x); %𝜃_1
q_1=sqrt(Pos_x^2+Pos_y^2); %q1
q_2=Pos_z-h; %q2
Psi_1=atan2d(q_2,q_1); %ψ_1
cbeta_num=f^2-e^2-q_2^2-q_1^2;
cbeta_den=-2*e*sqrt(q_1^2+q_2^2);
Cbeta=cbeta_num/cbeta_den; %cos(𝛽)
Sbeta=sqrt(1-Cbeta^2); %sin(𝛽)
Psi_2=atan2d(Sbeta,Cbeta); %ψ_2
th_2=Psi_1+Psi_2; % 𝜃_2
cphi_num=q_1^2+q_2^2-f^2-e^2;
cphi_den=-2*e*f;
CPhi=cphi_num/cphi_den; %cos(𝜑)
SPhi=sqrt(1-CPhi^2); %sin(𝜑)
Phi=atan2d(SPhi,CPhi); %𝜑
th_3= -(180-Phi); % 𝜃_3
ans1=[th_1,th_2,th_3];

%theta1 anticlockwise and elbow down
th_1=atan2d(Pos_y,Pos_x); %𝜃_1
q_1=sqrt(Pos_x^2+Pos_y^2); %q1
q_2=Pos_z-h; %q2
Psi_1=atan2d(q_2,q_1); %ψ_1
cbeta_num=f^2-e^2-q_2^2-q_1^2;
cbeta_den=-2*e*sqrt(q_1^2+q_2^2);
Cbeta=cbeta_num/cbeta_den; %cos(𝛽)
Sbeta=sqrt(1-Cbeta^2); %sin(𝛽)
Psi_2=atan2d(Sbeta,Cbeta); %ψ_2
th_2=Psi_1-Psi_2; % 𝜃_2
cphi_num=q_1^2+q_2^2-f^2-e^2;
cphi_den=-2*e*f;
CPhi=cphi_num/cphi_den; %cos(𝜑)
SPhi=sqrt(1-CPhi^2); %sin(𝜑)
Phi=atan2d(SPhi,CPhi); %𝜑
th_3=(180-Phi); % 𝜃_3
ans2=[th_1,th_2,th_3];

%theta 1 clockwise and elbow up
th_1=atan2d(-Pos_y,-Pos_x); %𝜃_1
q_1=sqrt(Pos_x^2+Pos_y^2); %q1
q_2=Pos_z-h; %q2
Psi_1=atan2d(q_2,q_1); %ψ_1
cbeta_num=f^2-e^2-q_2^2-q_1^2;
cbeta_den=-2*e*sqrt(q_1^2+q_2^2);
Cbeta=cbeta_num/cbeta_den; %cos(𝛽)
Sbeta=sqrt(1-Cbeta^2); %sin(𝛽)
Psi_2=atan2d(Sbeta,Cbeta); %ψ_2
psi=(Psi_1+Psi_2);
th_2=(180-psi); % 𝜃_2
cphi_num=q_1^2+q_2^2-f^2-e^2;
cphi_den=-2*e*f;
CPhi=cphi_num/cphi_den; %cos(𝜑)
SPhi=sqrt(1-CPhi^2); %sin(𝜑)
Phi=atan2d(SPhi,CPhi); % 𝜑
th_3=(180-Phi); % 𝜃_3
ans3=[th_1,th_2,th_3];

%theta 1 clockwise and elbow down
th_1=atan2d(-Pos_y,-Pos_x); %𝜃_1
q_1=sqrt(Pos_x^2+Pos_y^2); %q1
q_2=Pos_z-h; %q2
Psi_1=atan2d(q_2,q_1); %ψ_1
cbeta_num=f^2-e^2-q_2^2-q_1^2;
cbeta_den=-2*e*sqrt(q_1^2+q_2^2);
Cbeta=cbeta_num/cbeta_den; %cos(𝛽)
Sbeta=sqrt(1-Cbeta^2); %sin(𝛽)
Psi_2=atan2d(Sbeta,Cbeta); %ψ_2
psi=(Psi_2-Psi_1);
th_2=-(180-psi); % 𝜃_2
cphi_num=q_1^2+q_2^2-f^2-e^2;
cphi_den=-2*e*f;
CPhi=cphi_num/cphi_den; %cos(𝜑)
SPhi=sqrt(1-CPhi^2); %sin(𝜑)
Phi=atan2d(SPhi,CPhi); % 𝜑
th_3=-(180-Phi); % 𝜃_3
ans4=[th_1,th_2,th_3];
possiblesolutions=[ans1;ans2;ans3;ans4]