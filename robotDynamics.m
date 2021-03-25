function dx = robotDynamics(t, x, u, param)

% Extract states
pr  = x(1:2);   % Reduced momentum
q   = x(3:5);   % Displacement

dofIdx = param.dofIdx;      % Indices of degrees of freedom of interest

% Pad eta with zeros
eta6 = zeros(6,1);
eta6(dofIdx) = q;         	% N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
JK6 = eulerKinematicTransformation(eta6);
JK3 = JK6(dofIdx,dofIdx);

% Obtain rigid body mass matrix for DOF of interest
% M6 = rigidBodyMassMatrix(param);
M3 = param.M3;

% Get p from pr
p3 = robotMomentum(x,param);
p6 = zeros(6,1);
p6(dofIdx) = p3;

% Coriolis matrix for DOF of interest (as function of momentum)
C6 = rigidBodyCoriolisMatrixMomentum(p6);
C3 = C6(dofIdx,dofIdx);

% Get nu from p
nu3 = M3\p3;
nu6 = zeros(6,1);
nu6(dofIdx) = nu3;

% Coriolis matrix for DOF of interest (as function of velocity)
% C6 = rigidBodyCoriolisMatrixVelocity(nu6,param);
% C3 = C6(dofIdx,dofIdx);

% Damping matrix for DOF of interest
D6 = dampingMatrix(nu6,param);
D3 = D6(dofIdx,dofIdx);

% Constraint coefficient and annihilator
Bc = param.Bc;
Bcp = param.Bcp;
Mr = param.Mr;

% Skew symmetric interconnection matrix
JJ = [ ...
    -Bcp*C3*Bcp', -Bcp*JK3';
    JK3*Bcp', zeros(3,3)];

% Symmetric positive semi-definite total damping matrix
RR = [ ...
    Bcp*D3*Bcp', zeros(2,3);
    zeros(3,2), zeros(3,3)];

dHtildedx = [Mr\pr;zeros(3,1)];

Ba6 = param.Ba6;
Ba3 = Ba6(dofIdx,:);
G = [Bcp*Ba3; zeros(3,2)];

dx = (JJ - RR)*dHtildedx + G*u;
