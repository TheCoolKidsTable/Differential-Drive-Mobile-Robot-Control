% results = runtests('tests');
% assert(all([results.Passed]));

% Robot parameters
param = robotParameters();

% Simulation parameters
param.dt    = 0.05;         % Evaluation time interval (simulation may internally use smaller steps) [s]
param.T     = 32;         	% Total simulation time [s]
tHist       = 0:param.dt:param.T;       % Specify times the output is returned
options     = odeset('MaxStep',0.005);

% Set initial conditions
x0 = robotInitialConditions(param);

% Load trajectory
load('traj.mat','traj');
param.traj = traj;

uHist = nan(2,length(tHist));

% Run the simulation
mode = 'MPC';
% mode = 'PHS';
% mode = 'MPC';
switch mode
    case 'test'
        func = @(t,x) robotDynamics(t,x,inputTorque(t,x),param);
        [tHist,xHist] = ode45(func,tHist,x0,options);
        tHist = tHist.';
        xHist = xHist.';
        for i = 1:length(tHist)
            uHist(:,i) = inputTorque(tHist(i),xHist(:,i));
        end
    case 'PHS'
        func = @(t,x) robotDynamics(t,x,controlPHS(t,x,param),param);
        [tHist,xHist] = ode45(func,tHist,x0,options);
        tHist = tHist.';
        xHist = xHist.';
        for i = 1:length(tHist)
            uHist(:,i) = controlPHS(tHist(i),xHist(:,i),param);
        end
    case 'MPC'
        % Remove any stuck waitbars
        delete(findall(0,'tag','TMWWaitbar'));
        
        wh = waitbar(0,getStatusMsg, ...
            'Name', 'MCHA4100 Lab 3', ...
            'CreateCancelBtn', 'setappdata(gcbf,''cancelling'',1)');
        
        U = [];
        u = [0.1;0.1];                      % Initial stored control action
        uHist(:,1) = u;
        xHist = zeros(5,length(tHist));
        for i = 1:length(tHist)-1
            if getappdata(wh,'cancelling')  % Check if waitbar cancel button clicked
                delete(wh);
                error('User cancelled operation.')
            end
            
            try % you can't succeed if you don't
                % Pretend to apply stored control action u
                % PRETENDING

                % Simulate one time step with ZOH input u
                func = @(t,x) robotDynamics(t,x,u,param);
                [~,xTemp] = ode45(func,[tHist(i) tHist(i+1)],xHist(:,i),options);
                xTemp = xTemp.';
                xHist(:,i+1) = xTemp(:,end);

                % Pretend to take measurements from sensors
                % Pretend to estimate next state
                % PRETENDING INTENSIFIES

                % Compute next control action
                U = controlMPC(tHist(i+1),xHist(:,i+1),u,U,param);

                % Store next control action to apply
                u = U(1:2);

                uHist(:,i+1) = u; % Save for plotting
            catch hot_potato
                delete(wh);                 % Remove waitbar if error
                rethrow(hot_potato);        % Someone else's problem now
            end
            waitbar(i/length(tHist),wh);    % Update waitbar
        end
        delete(wh);                         % Remove waitbar if we complete successfully
    otherwise
        error('Unknown mode');
end

%% Plot history
NHist   = xHist(3,:);
EHist   = xHist(4,:);
psiHist = xHist(5,:);
gHist   = nan(1,length(tHist));
HHist   = nan(1,length(tHist));
nu3Hist = nan(3,length(tHist));
Xtraj = nan(3,2,length(tHist));
Bc = robotConstraints(param);
for i = 1:length(tHist)
    nu3Hist(:,i) = robotVelocity(xHist(:,i),param);
    gHist(i) = Bc.'*nu3Hist(:,i);
    HHist(i) = robotEnergy(xHist(:,i),param);
    Xtraj(:,:,i) = trajectoryEval(param.traj,tHist(i));
end

Ntraj = squeeze(Xtraj(1,1,:)).';
Etraj = squeeze(Xtraj(2,1,:)).';

figure(1);clf

subplot(3,3,1)
plot(tHist,NHist)
grid on
title('North position')
ylabel('N [m]')
subplot(3,3,4)
plot(tHist,EHist)
grid on
title('East position')
ylabel('E [m]')
subplot(3,3,7)
plot(tHist,psiHist*180/pi)
grid on
title('Yaw angle')
xlabel('Time [s]')

ylabel('r [\circ]')
subplot(3,3,2)
plot(tHist,nu3Hist(1,:))
grid on
title('Surge velocity')
ylabel('u [m/s]')
subplot(3,3,5)
plot(tHist,nu3Hist(2,:))
grid on
title('Sway velocity')
ylabel('v [m/s]')
subplot(3,3,8)
plot(tHist,nu3Hist(3,:)*180/pi)
grid on
title('Yaw angular velocity')
ylabel('r [\circ/s]')
xlabel('Time [s]')

subplot(4,3,3)
plot(tHist,uHist(1,:))
grid on
title('Left wheel torque')
ylabel('\tau [N.m]')
subplot(4,3,6)
plot(tHist,uHist(2,:))
grid on
title('Right wheel torque')
ylabel('\tau [N.m]')
subplot(4,3,9)
plot(tHist,HHist)
grid on
title('Total energy')
ylabel('H [J]')
subplot(4,3,12)
plot(tHist,gHist)
grid on
title('Nonholonomic constraint violation')
ylabel('B_c^T \nu [-]')
xlabel('Time [s]')

%% Animation

outputVideo = false;

if outputVideo
    vid = VideoWriter(['lab3' mode '.mp4'],'MPEG-4');
    vid.FrameRate = 1/param.dt;
    vid.Quality = 100;
    open(vid);
end

fig     = 2;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf,'FontSize',14);
hold(ax,'on');

dx = 0.1;
colormap(ax,'gray');
Egridlines = 1:dx:4;
Ngridlines = 0.85:dx:3.25;
[Egrid, Ngrid] = meshgrid(Egridlines,Ngridlines);
if strcmp(mode,'test')
    Cgrid = ones(size(Egrid));
else
    Cgrid = zeros(size(Egrid));
end
caxis(ax,[0 1]);

hGrid   = pcolor(ax,Egrid,Ngrid,Cgrid);
hGrid.EdgeColor = 'none';
hGrid.FaceAlpha = 0.5;

hLt     = plot(ax,nan(size(EHist)),nan(size(NHist)),'r');
hRt     = plot(ax,nan(size(EHist)),nan(size(NHist)),'g');
hSt     = plot(ax,nan(size(EHist)),nan(size(NHist)),'b');
htraj   = plot(ax,Etraj,Ntraj,'k:');
hT      = plot(ax,nan,nan,'ro');
hP      = plot(ax,nan,nan,'k-');
hL      = plot(ax,0,0,'r.');
hR      = plot(ax,0,0,'g.');
hS      = plot(ax,0,0,'b.');
hB      = plot(ax,0,0,'k.');
hC      = plot(ax,0,0,'k.');
tL      = text(ax,0,0,' L','FontSize',10,'Color','r');
tR      = text(ax,0,0,' R','FontSize',10,'Color','g');
tS      = text(ax,0,0,' S','FontSize',10,'Color','b');
tB      = text(ax,0,0,' B','FontSize',10,'Color','k');
tC      = text(ax,0,0,' C','FontSize',10,'Color','k');

hold(ax,'off');
axis(ax,'equal');
axis(ax,[min([EHist,Etraj])-0.5,max([EHist,Etraj])+0.5,min([NHist,Ntraj])-0.5,max([NHist,Ntraj])+0.5]);
% grid(ax,'on');
xlabel(ax,'East [m]');
ylabel(ax,'North [m]');

a = 0.15;
r = 0.2;
theta = linspace(3*pi/2,pi/2,50);
rPCb = [ [0;r;0], [a;r;0], [a;-r;0], [0;-r;0], r*[cos(theta);sin(theta);zeros(size(theta))] ];
Se3  = skew([0;0;1]);

rBNn = nan(3,length(tHist));
rCNn = nan(3,length(tHist));
rLNn = nan(3,length(tHist));
rRNn = nan(3,length(tHist));
rSNn = nan(3,length(tHist));
rANn = nan(3,length(tHist));
for i = 1:length(tHist)
    Rnb = expm(psiHist(i)*Se3);
    rBNn(:,i) = [NHist(i); EHist(i); 0];
    rCNn(:,i) = rBNn(:,i) + Rnb*param.rCBb;
    rLNn(:,i) = rBNn(:,i) + Rnb*param.rLBb;
    rRNn(:,i) = rBNn(:,i) + Rnb*param.rRBb;
    rSNn(:,i) = rBNn(:,i) + Rnb*param.rSBb;
    
    rPNn = rCNn(:,i) + Rnb*rPCb;
    hP.XData = rPNn(2,:);
    hP.YData = rPNn(1,:);
    
    hLt.XData = rLNn(2,:);
    hLt.YData = rLNn(1,:);
    hRt.XData = rRNn(2,:);
    hRt.YData = rRNn(1,:);
    hSt.XData = rSNn(2,:);
    hSt.YData = rSNn(1,:);
    
    hT.XData = Etraj(i);
    hT.YData = Ntraj(i);
    
    hL.XData = rLNn(2,i);
    hL.YData = rLNn(1,i);
    tL.Position = [rLNn(2,i),rLNn(1,i),0];
    hR.XData = rRNn(2,i);
    hR.YData = rRNn(1,i);
    tR.Position = [rRNn(2,i),rRNn(1,i),0];
    hS.XData = rSNn(2,i);
    hS.YData = rSNn(1,i);
    tS.Position = [rSNn(2,i),rSNn(1,i),0];
    hB.XData = rBNn(2,i);
    hB.YData = rBNn(1,i);
    tB.Position = [rBNn(2,i),rBNn(1,i),0];
    hC.XData = rCNn(2,i);
    hC.YData = rCNn(1,i);
    tC.Position = [rCNn(2,i),rCNn(1,i),0];
    
    if ~strcmp(mode,'test')
    
        for alpha = linspace(0,1,5)
            rANn = (1-alpha)*rLNn(:,i) + alpha*rRNn(:,i);
            Nidx = find(Ngridlines <= rANn(1), 1, 'last');
            Eidx = find(Egridlines <= rANn(2), 1, 'last');
            if ~isempty(Nidx) && ~isempty(Eidx)
                hGrid.CData(Nidx,Eidx) = 1;
            end
        end
    
        drawnow
        if outputVideo
            writeVideo(vid,getframe(hf));
        else
            pause(param.dt);
        end 
    end
end

if outputVideo
    close(vid);
end

