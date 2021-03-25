function p3 = robotMomentum(x,param)

% Extract states
pr = x(1:2);   % Reduced momentum

% Constraint coefficient and annihilator
% [Bc,Bcp] = robotConstraints(param);

% Obtain rigid body mass matrix for DOF of interest
% M6 = rigidBodyMassMatrix(param);
% M3 = M6(param.dofIdx,param.dofIdx);

% Compute p from pr
A = param.matrix;
 
p3 = A\[0;pr];

