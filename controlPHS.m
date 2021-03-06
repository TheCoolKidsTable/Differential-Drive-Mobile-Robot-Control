function u = controlPHS(t, x, param)

% Extract states
pr  = x(1:2);   % Reduced momentum
q   = x(3:5);   % Displacement

dofIdx = param.dofIdx;

% Evaluate trajectory
X = trajectoryEval(param.traj,t);
rPNn = [X(1:2,1);0];
vPNn = [X(1:2,2);0];

Bcp = param.Bcp; %2x3
Ba3 = param.Ba6(dofIdx,:);%3x2

eta6 = zeros(6,1);
eta6(dofIdx) = q; 
JK6 = eulerKinematicTransformation(eta6);
JK3 = JK6(dofIdx,dofIdx);

kd = 500;
dd = 150;
T = 0.3;


dHd_dq = [kd*(T*cos(q(3)) + q(1) - rPNn(1));
          kd*(T*sin(q(3)) + q(2) - rPNn(2));
          kd*(T*cos(q(3)) + q(1) - rPNn(1))*(-T*sin(q(3))) + ... 
          kd*(T*sin(q(3)) + q(2) - rPNn(2))*(T*cos(q(3)));
];

A = param.matrix;
p3 = A\[0;pr];
M3 = param.M3;
nu3 = M3\p3;
nu6 = zeros(6,1);
nu6(dofIdx) = nu3;
D6 = dampingMatrix(nu6,param);
D3 = D6(dofIdx,dofIdx);
Mr = param.Mr;

rTBb = [T;0;0];

Dd = [dd*eye(3) dd*skew(-rTBb);
        dd*skew(rTBb) skew(rTBb)*dd*skew(-rTBb)];    
Dd3 = Dd(dofIdx,dofIdx);

Rnb = eulerRotation([0;0;q(3)]);
vPNb = Rnb'*vPNn;
bd = dd*[-vPNb;-skew(rTBb)*vPNb];
bd3 = bd(dofIdx);

% Control law
u = (Bcp*Ba3)\((Bcp*D3*Bcp' - Bcp*Dd3*Bcp')*(Mr\pr) - (Bcp*JK3'*dHd_dq) - Bcp*bd3);
