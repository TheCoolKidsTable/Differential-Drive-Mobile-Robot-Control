function Ba = actuatorConfiguration(param)
%ACTUATORCONFIGURATION Actuator configuration matrix maps actuator forces
%into body forces
%
% tau = Ba * u
%

rw      = param.rw;
rLBb    = param.rLBb;
rRBb    = param.rRBb;

Ba = [1/rw, 1/rw;
      0,0;
      0,0;
      0,0;
      0,0;
      param.d/(2*rw), -param.d/(2*rw);
];