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
% syms L1 L2 theta1 theta2 real;
% J = [-L1*cos(theta1)-L2*cos(theta1+theta2),-L2*cos(theta1 + theta2);L1*sin(theta1)+L2*sin(theta1+theta2),L2*sin(theta1 + theta2)];
% pretty(inv(J));

% 测试信任度函数
% x = 0:0.01:1;
% w = 0.1;
% w1 = 0.2;
% Cf = 0.5*(1./(1+exp(-(4.*x/w-2)))+1./(1+exp(-(4.*(1-x)/w-2))));
% Cf1 = 0.5*(1./(1+exp(-(4.*x/w1-2)))+1./(1+exp(-(4.*(1-x)/w1-2))));
% plot(x,Cf,x,Cf1);

% 高度信任度
x = -1:0.01:1;
k1 = 50;
k2 = 10;
Cz = zeros(size(x));
for i  = 1:length(x)
    if x(i) > 0
        Cz(i) = exp(-k1*x(i)^2);
    else
        Cz(i) = exp(-k2*x(i)^2);
    end
end
plot(x,Cz);