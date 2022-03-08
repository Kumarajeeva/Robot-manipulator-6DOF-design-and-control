%Forward dynamics of the Robot Manipulator

%Computing the forward dynamics of the Microbot manipulator using the inputs as the torque profiles obtained from
the inverse dynamics program. Plotting the joint positions, velocities, accelerations and torques against time

clear all
close all
clc
syms a0 a1 a2 a3 alpha0 alpha1 alpha2 alpha3 d1 d1 d2 d4 theta1 theta2 theta3 theta4 tdot1 tdot2 tdot3 tdot4 tddot1 tddot2 tddot3 L1 L2 L3 M1 M2 M3 t

%D-H parameters
a0 = 0;a1 = 0;a2 = 3;a3 = 4;
d1 = 2;d2 = 0;d3 = 0;d4 = 0;
alpha0 = 0;alpha1 = pi/2;alpha2 = 0;alpha3 = 0;
L1 = 0;L2 = 3;L3 = 4;

Torque1 = sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)^2*(32*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 + (pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) - (392*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/5 + 32*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 - (pi*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) + (3*pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/2 + 6*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3) + (243388915243820059990639815496725*pi^2*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/162259276829213363391578010288128 + 4*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2 + (162259276829213368359335610309639*pi^2*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/20282409603651670423947251286016) - (16524327490667539036901295542150738761909780466889*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/13164036458569648337239753460458804039861886925068638906788872192 - cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(24*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 + (pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) - (588*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/5 + 24*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 - (pi*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) + cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(32*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 + (pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) - (392*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/5 + 32*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*((pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/16 - (pi*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/4) + (3*pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/2 + 6*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3) + (243388915243820059990639815496725*pi^2*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/162259276829213363391578010288128 + 4*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2 + (162259276829213368359335610309639*pi^2*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/20282409603651670423947251286016) + (9*pi^2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2)/4 + 9*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3) + (730166745731460179971919446490175*pi^2*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/324518553658426726783156020576256 + 3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))^2 + (486777830487640105078006830928917*pi^2*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/81129638414606681695789005144064) - (4967757600021511*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16))/10141204801825835211973625643008 + (4967757600021511*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) +
1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16))/10141204801825835211973625643008 - (14903272800064533*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + 2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + 2*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*((pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)^2*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/4 - (pi*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)^2*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/4) + (26328072917139298286609018205555527386232636944904737985456310321*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/13164036458569648337239753460458804039861886925068638906788872192))/81129638414606681695789005144064 + (3627756489168007981517746420298787*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/16455045573212060421549691825573505049827358656335798633486090240 - (14903272800064533*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + (162259276829213368359335610309639*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/40564819207303340847894502572032 - 2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) - pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3))))/81129638414606681695789005144064 - (730260367203162117*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/202824096036516704239472512860160 + (44709818400193599*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/649037107316853453566312041152512 + (4967757600021511*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/20282409603651670423947251286016;

Torque2 = 8*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) +
1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) - (3326315174998874073721289211444399*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/162259276829213363391578010288128 - 8*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) - 3*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + 2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + 2*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*((pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)^2*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/4 - (pi*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)^2*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/4) + (26328072917139298286609018205555527386232636944904737985456310321*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/13164036458569648337239753460458804039861886925068638906788872192) + (730260367203162117*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/202824096036516704239472512860160 - 3*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*(2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + (162259276829213368359335610309639*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/40564819207303340847894502572032 - 2*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) - pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3))) - (294*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 + (9*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/8 + 4*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3));

Torque3 = 8*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((19746054687854474924053897117645122037479483886978449819495241875*pi^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/105312291668557186697918027683670432318895095400549111254310977536 - (795070456463145529302756730622635*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4))/81129638414606681695789005144064 + (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) - (162259276829213368359335610309639*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/10141204801825835211973625643008 - 8*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*((243388915243820059990639815496725*pi*(50*exp(-(5*t)/2) - (200*exp(-(10*t)/3))/3))/324518553658426726783156020576256 - (243420122401054039*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/405648192073033408478945025720320 + (49*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2)/5 - (3*pi^2*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3)))/16) + 4*pi*cos((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*sin((pi*(6*exp(-(10*t)/3) - 8*exp(-(5*t)/2) + 1))/4)*(pi*cos((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)) - pi*sin((pi*(6*exp(-(10*conj(t))/3) - 8*exp(-(5*conj(t))/2) + 1))/4)^2*(20*exp(-(5*conj(t))/2) - 20*exp(-(10*conj(t))/3)))*(20*exp(-(5*t)/2) - 20*exp(-(10*t)/3));

%mass and gravity
M1 = 4;M2 = 2;M3 = 2;
g = 0;
t0_1 = [cos(theta1) -sin(theta1) 0 a0; sin(theta1)*cos(alpha0) cos(theta1)*cos(alpha0) -sin(alpha0) -sin(alpha0)*d1; sin(theta1)*sin(alpha0) cos(theta1)*sin(alpha0) cos(alpha0) cos(alpha0)*d1;0 0 0 1];
t1_2 = [cos(theta2) -sin(theta2) 0 a1; sin(theta2)*cos(alpha1) cos(theta2)*cos(alpha1) -sin(alpha1) -sin(alpha1)*d2; sin(theta2)*sin(alpha1) cos(theta2)*sin(alpha1) cos(alpha1) cos(alpha1)*d2;0 0 0 1];
t2_3 = [cos(theta3) -sin(theta3) 0 a2; sin(theta3)*cos(alpha2) cos(theta3)*cos(alpha2) -sin(alpha2) -sin(alpha2)*d3; sin(theta3)*sin(alpha2) cos(theta3)*sin(alpha2) cos(alpha2) cos(alpha2)*d3;0 0 0 1];
t3_4 = [cos(theta4) -sin(theta4) 0 a3; sin(theta4)*cos(alpha3) cos(theta4)*cos(alpha3) -sin(alpha3) -sin(alpha3)*d4; sin(theta4)*sin(alpha3) cos(theta4)*sin(alpha3) cos(alpha3) cos(alpha3)*d4;0 0 0 1];
Tb_w = t0_1*t1_2*t2_3*t3_4; %Transformation matrix


%for outward iterations
r1_0 = transpose(t0_1(1:3,1:3));
r2_1 = transpose(t1_2(1:3,1:3));
r3_2 = transpose(t2_3(1:3,1:3));
r4_3 = transpose(t3_4(1:3,1:3));


%for inward iterations
r0_1 = t0_1(1:3,1:3);
r1_2 = t1_2(1:3,1:3);
r2_3 = t2_3(1:3,1:3);
r3_4 = t3_4(1:3,1:3);


%position and centre of mass position
p0_1 = [0;0;2];
p1_2 = [0;0;0];
p2_3 = [3;0;0];
p3_4 = [4;0;0];
pc1_1 = [0;0;0];
pc2_2 = [3;0;0];
pc3_3 = [4;0;0];


%Assumptions
IC11 = zeros(3,3);
IC22 = zeros(3,3);
IC33 = zeros(3,3);
vd0_0 = [0;g;0];
f4_4 = zeros(3,1);
n4_4 = zeros(3,1);
w0_0 = zeros(3,1);
wd0_0 = zeros(3,1);


%OUTWARD ITERATIONS

%link 1
w1_1 = r1_0*w0_0 + [0;0;tdot1]; %angular velocity
wd1_1 = r1_0*wd0_0 + cross(r1_0*w0_0,[0;0;tdot1]) + [0;0;tddot1]; %angular acceleration
vd1_1 = r1_0*(cross(wd0_0,p0_1) + cross(w0_0 , cross(w0_0,p0_1)) + vd0_0); %linear acceleration
vdc1_1 = cross(wd1_1,pc1_1) + cross(w1_1, cross(w1_1,pc1_1)) + vd1_1; %linear acceleration of centre of mass
F1_1 = M1*vdc1_1; %force exerted
N1_1 = IC11*wd1_1 + cross(w1_1,IC11*w1_1); %moment exerted

%link 2
w2_2 = r2_1*w1_1 + [0;0;tdot2];
wd2_2 = r2_1*wd1_1 + cross(r2_1*w1_1,[0;0;tdot2]) + [0;0;tddot2];
vd2_2 = r2_1*(cross(wd1_1,p1_2) + cross(w1_1 , cross(w1_1,p1_2)) + vd1_1);
vdc2_2 = cross(wd2_2,pc2_2) + cross(w2_2, cross(w2_2,pc2_2)) + vd2_2;
F2_2 = M2*vdc2_2;
N2_2 = IC22*wd2_2 + cross(w1_1,IC11*w1_1);

%link 3
w3_3 = r3_2*w2_2 + [0;0;tdot3];
wd3_3 = r3_2*wd2_2 + cross(r3_2*w2_2,[0;0;tdot3]) + [0;0;tddot3];
vd3_3 = r3_2*(cross(wd2_2,p2_3) + cross(w2_2 , cross(w2_2,p2_3)) + vd2_2);
vdc3_3 = cross(wd3_3,pc3_3) + cross(w3_3, cross(w3_3,pc3_3)) + vd3_3;
F3_3 = M3*vdc3_3;
N3_3 = IC33*wd3_3 + cross(w2_2,IC22*w2_2);


%INWARD ITERATIONS

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
Torque1 = transpose(n1_1)*[0;0;1];
Torque2 = transpose(n2_2)*[0;0;1];
Torque3 = transpose(n3_3)*[0;0;1]; %Obtaining torques in terms of symbol

%for calculating M
variables = [tddot1,tddot2,tddot3];
Torque = [Torque1,Torque2,Torque3];
[M1,xx] = equationsToMatrix(Torque,variables); %Obtaining M matrix
Th_1 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
Th_2 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
Th_3 = deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
TH=[Th_1;Th_2;Th_3];
Thd_1 = gradient(Th_1);
Thd_2 = gradient(Th_2);
Thd_3 = gradient(Th_3);
Thdd_1 = gradient(Thd_1);
Thdd_2 = gradient(Thd_2);
Thdd_3 = gradient(Thd_3);

%Forward dynamics
M2 = subs(M1,theta3,Th_3);
Minv = inv(M2);
VplusG = subs(Torque,[tddot1,tddot2,tddot3],[0,0,0]);
VplusG_sub = subs(VplusG,[theta1,theta2,theta3,tdot1,tdot2,tdot3],[Th_1,Th_2,Th_3,Thd_1,Thd_2,Thd_3]);
eqn = Torque - VplusG_sub;
acceleration = eqn*Minv;
time = 0:0.05:2;

%Acceleration
accelaration_numerical = subs(acceleration,[theta1,theta2,theta3,tdot1,tdot2,tdot3,tddot1,tddot2,tddot3],[Th_1,Th_2,Th_3,Thd_1,Thd_2,Thd_3,Thdd_1,Thdd_2,Thdd_3]);
accel_time=subs(accelaration_numerical,t,time);
accel_plot=vpa(subs(accelaration_numerical,t,time));
ac_1=accel_plot(1:41);ac_2=accel_plot(42:82);ac_3=accel_plot(83:123);
AC=[ac_1;ac_2;ac_3]
figure(1)
subplot(2,2,1)
plot(time,ac_1,'r')
title('Acceleration for joint 1')
subplot(2,2,2)
plot(time,ac_2,'g')
title('Acceleration for joint 2')
subplot(2,2,3)
plot(time,ac_3,'b')
title('Acceleration for joint 3')
%Integration
th=deg2rad(45)*(1+6*exp(-t/0.3)-8*exp(-t/0.4));
th_d=diff(th);
th_dd=diff(th_d);
th_init=subs(th,t,0); %Initial condition for theta
thd_init=subs(th_d,t,0); %Initial condition for thetadot
thdd_init=subs(th_dd,t,0); %Initial condition for thetadoubledot
syms t;
T(1)=th_init;
T_dot(1)=thd_init;
time=linspace(0,2,41);

%Euler's integration method
for i=1:length(time)-1
T(i+1)=T(i)+T_dot(i)*time(i)+0.5*AC(1,i)*(time(i)^2);
T_dot(i+1)=T_dot(i)+AC(1,i)*time(i);
end
z=linspace(0,2,41);

%Plotting Velocity
vel=int(accelaration_numerical,t);
vel_ans=vpa(subs(vel,t,z))
pos_z=0.8;
figure(2)
subplot(2,2,1)
plot(z,vel_ans(1:41),'r')
title('Velocity for joint 1')
hold on
subplot(2,2,2)
plot(z,vel_ans(42:82),'b')
title('Velocity for joint 2')
subplot(2,2,3)
plot(z,vel_ans(83:123),'k')
title('Velocity for joint 3')
hold off

%Plotting Position
pos=int(vel,t);
pos_ans=vpa(subs(pos,t,z))
pos_final=pos_z+pos_ans;
figure(3)
subplot(2,2,1)
plot(z,pos_final(1:41),'k')
title('Position for joint 1')
hold on
subplot(2,2,2)
plot(z,pos_final(42:82),'r')
title('Position for joint 2')
subplot(2,2,3)
plot(z,pos_final(83:123),'b')
title('Position for joint 3')
hold off

%Plotting torques
figure(4)
subplot(2,2,1)
tou1_plot=vpa(subs(tou1,t,z));
plot(z,tou1_plot,'k')
title('Torque for joint 1')
hold on
subplot(2,2,2)
tou2_plot=vpa(subs(tou2,t,z));
plot(z,tou2_plot,'g')
title('Torque for joint 2')
subplot(2,2,3)
tou3_plot=vpa(subs(tou3,t,z));
plot(z,tou3_plot,'b')
title('Torque for joint 3')
hold off