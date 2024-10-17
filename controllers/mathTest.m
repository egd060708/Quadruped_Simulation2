clc;
clear;

% theta1 = 0.1;
% theta2 = -0.1;
% L1 = 0.09;
% L2 = 0.1;
% J = [-L1*cos(theta1),-L2*cos(theta2);L1*sin(theta1),L2*sin(theta2)];
% Ji = [-(1/(L1*cos(theta1))+cos(theta2)*tan(theta1)/(L1*sin(theta2-theta1))),-cos(theta2)/(L1*sin(theta2-theta1));...
%     sin(theta1)/(L2*sin(theta2-theta1)),cos(theta1)/(L2*sin(theta2-theta1))];
% disp(inv(J));
% disp(Ji);

% 符号表达式求逆
syms L1 L2 theta1 theta2 real;
J = [-L1*cos(theta1)-L2*cos(theta1+theta2),-L2*cos(theta1 + theta2);L1*sin(theta1)+L2*sin(theta1+theta2),L2*sin(theta1 + theta2)];
pretty(inv(J));