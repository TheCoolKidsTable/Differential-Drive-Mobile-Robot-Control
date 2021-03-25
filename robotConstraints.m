function [Bc,Bcp] = robotConstraints(param)

l = param.l;

Bc = [0;1;l];    % Implement this in Problem 1

Bcp = [2 l -1; 1 -l 1];   % Implement this in Problem 5
