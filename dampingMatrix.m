  function D = dampingMatrix(nu, param)

vBNb = nu(1:3);
omegaBNb = nu(4:6);

SrSBb = skew(param.rSBb);

vSNb = vBNb + cross(omegaBNb,param.rSBb);

mu = param.mu;
wS = skidNormalForce(param);

% To implement pure Coulomb friction, we could implement the following:
%{
if norm(vSNb) == 0
    beta = 0;
else
    beta = mu*wS/norm(vSNb);
end
%}
% Unfortunately, the above method causes the ODE solver run very slowly
% since it has to use very small step sizes to handle a large damping
% coefficient beta for small velocity. To speed up the simulation, we use a
% small threshold for velocity, below which we switch to a viscous friction
% model during the lockup phase, which is implemented as follows:
beta = mu*wS/max(norm(vSNb),1e-3);

D = beta*[eye(3), -SrSBb;
          SrSBb,-SrSBb*SrSBb];
