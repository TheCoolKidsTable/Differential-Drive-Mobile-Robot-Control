function e = errorMPC(t1, x1, u0, U, param) %#ok<INUSL>

dt = param.dt;
N = param.nHorizon; % Control horizon (set in controlMPC.m, not here)

% Penalty coefficients
sqrtqr      = 50;
sqrtqpsi    = 1e-1;
sqrtru      = 1;

e = zeros(5*N,1);

t = t1;
x = x1;
f = @robotDynamics;

for i = 1:N                 % for each step in the control horizon
    u = U(2*i-1:2*i);
    
    % Do one step of RK4 integration
    f1 = f(t,        x,           u, param);
    f2 = f(t + dt/2, x + f1*dt/2, u, param);
    f3 = f(t + dt/2, x + f2*dt/2, u, param);
    f4 = f(t + dt,   x + f3*dt,   u, param);
    x = x + (f1 + 2*f2 + 2*f3 + f4)*dt/6;
    t = t + dt;
    
    % Evaluate trajectory
    X = trajectoryEval(param.traj,t);
    rPNn = [X(1:2,1);0];
    psistar = X(3,1);
    
    % Extract states
    q = x(3:5);         % Displacement states
    rABn = [param.l;0;0];
    Rnb = eulerRotation([0;0;q(3)]);
    rBNn = [q(1);q(2);0];
    rANn = rBNn + Rnb*rABn;

    % Error for current step
    e((i-1)*5+1:i*5) = [sqrtqr* (rANn(1:2)-rPNn(1:2)); sqrtqpsi*(q(3)-psistar); sqrtru*u];
end