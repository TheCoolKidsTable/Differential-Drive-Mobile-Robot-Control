function U = controlMPC(t1,x1,u0,U,param)

m = size(u0,1);

param.nHorizon = 8; % Control horizon

if isempty(U)
    U = [u0;zeros(m*(param.nHorizon-1),1)];
else
    % warm start
    U = [U(m+1:end);zeros(m,1)];
end

options = optimoptions('lsqnonlin','Display','iter');
err = @(U) errorMPC(t1,x1,u0,U,param);
U = lsqnonlin(err,U,[],[],options);
