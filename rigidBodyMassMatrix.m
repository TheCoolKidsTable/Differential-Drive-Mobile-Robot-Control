function MRB = rigidBodyMassMatrix(param)
% rigidBodyMassMatrix Calculates the rigid body mass matrix

fname   = "rigidBodyMassMatrix";
validateattributes(param, {'struct'}, {'scalar'}, fname, "param", 2);

m       = param.m; 
IBb     = param.IBb;
rCBb    = param.rCBb;
SrCBb   = skew(rCBb);

% Generalised mass matrix (constant in body coordinates)
MRB = [ ...
    m*eye(3), -m*SrCBb;
    m*SrCBb, IBb];