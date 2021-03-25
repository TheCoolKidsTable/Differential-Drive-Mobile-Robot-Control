function C = rigidBodyCoriolisMatrixMomentum(p)

% Coriolis matrix in terms of body-fixed momentum
pv = p(1:3,:);
pw=p(4:6,:);

C = [ ...
    zeros(3),-skew(pv);
    -skew(pv), -skew(pw)];