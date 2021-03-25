function param = robotParameters()

param.l     = 0.2;          % Longitudinal distance from B to wheels [m]
param.d     = 0.3;          % Distance between wheels [m]
param.c     = 0.1;          % Longitudinal distance from B to C [m]
param.s     = 0.05;         % Longitudinal distance from B to S [m]
param.rw    = 0.04;         % Wheel radius [m]
param.rLBb	= [param.l;-param.d/2;0];   % Implement this in Problem 1
param.rRBb 	= [param.l;param.d/2;0];   % Implement this in Problem 1
param.rCBb	= [param.c;0;0];   % Implement this in Problem 3
param.rSBb 	= [-param.s;0;0];   % Implement this in Problem 4

param.m     = 3;            % Robot mass [kg]
param.Iz    = 3*0.15^2;     % Yaw inertia [kg.m^2]
param.g     = 3.71;         % Acceleration due to gravity on Mars [m/s^2]
param.IBb  	= diag([0,0,param.Iz]);

param.mu    = 0.2;          % Coefficient of kinetic friction for skid [-]

param.dofIdx = [1 2 6];     % N, E, psi

M6 = rigidBodyMassMatrix(param);
param.M3 = M6(param.dofIdx,param.dofIdx);

[Bc,Bcp] = robotConstraints(param);
param.Bc = Bc;
param.Bcp = Bcp;

param.Mr = Bcp*param.M3*Bcp';

Ba6 = actuatorConfiguration(param);
param.Ba6 = Ba6;

param.matrix = [Bc.'/param.M3; Bcp];

