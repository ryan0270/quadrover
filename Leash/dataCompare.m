clear

%% Get vicon data
DELIMITER = ',';
HEADERLINES = 1;

% Import the file
newData1 = importdata('runData/data.csv', DELIMITER, HEADERLINES);

% Create new variables in the base workspace from those fields.
vars = fieldnames(newData1);
for i = 1:length(vars)
    assignin('base', vars{i}, newData1.(vars{i}));
end

dataVicon = data;

frame           = dataVicon(:,1)';
flightMode      = dataVicon(:,2)';
timeVicon       = dataVicon(:,3)'/1000;
commandVals     = dataVicon(:,4:7)';
targetPos       = dataVicon(:,8:19)';
stateVicon      = dataVicon(:,20:31)';
stateFiltered	= dataVicon(:,32:43)';

% targetPos(1:6,:) = targetPos(1:6,:)*180/pi;
% stateVicon(1:6,:) = stateVicon(1:6,:)*180/pi;
% stateFiltered(1:6,:) = stateFiltered(1:6,:)*180/pi;

% pathStartIndex = find(flightMode==1,1,'first');
% if isempty(pathStartIndex)
%     pathStartIndex = 1;
% end
% timeVicon = time-time(pathStartIndex);

%% Get phone data
phoneFile = 'phoneLog.txt';
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');
stateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1001);
stateRefTime = phoneData(stateRefIndices,1)'/1000;
stateRef = phoneData(stateRefIndices,3:end)';
stateRef_dt = mean(diff(stateRefTime));

angleStateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1001);
angleStateRefTime = phoneData(angleStateRefIndices,1)'/1000;
angleStateRef = phoneData(angleStateRefIndices,3:8)';

angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1002);
angleStateTime = phoneData(angleStateIndices,1)'/1000;
angleState = phoneData(angleStateIndices,3:8)';

tranStateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1011);
tranStateRefTime = phoneData(tranStateRefIndices,1)'/1000;
tranStateRef = phoneData(tranStateRefIndices,3:8)';

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1012);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

tranStateRefInterp = interp1(tranStateRefTime,tranStateRef',angleStateRefTime)';
tranStateInterp = interp1(tranStateTime,tranState',angleStateTime)';
stateRefTime = angleStateRefTime;
stateRef = [angleStateRef; tranStateRefInterp];
stateTime = angleStateTime;
state = [angleState; tranStateInterp];
state_dt = mean(diff(stateTime));

gyroBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1003);
gyroBiasTime = phoneData(gyroBiasIndices,1)'/1000;
gyroBias = phoneData(gyroBiasIndices,3:5)';
gyroBias_dt = mean(diff(gyroBiasTime));

% innovationIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -800);
% innovationTime = phoneData(innovationIndices,1)'/1000;
% innovation = phoneData(innovationIndices,3:8)';
% innovation_dt = mean(diff(innovationTime));

yawResetIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -805);
yawResetTime = phoneData(yawResetIndices,1)'/1000;
yawResetMag = phoneData(yawResetIndices,3:end)';

uBIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -801);
uBTime = phoneData(uBIndices,1)'/1000;
uB = phoneData(uBIndices,3:5)';
uB_dt = mean(diff(uBTime));

vBIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -803);
vBTime = phoneData(vBIndices,1)'/1000;
vB = phoneData(vBIndices,3:5)';
vB_dt = mean(diff(vBTime));

nemoAttIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 11);
nemoAttTime = phoneData(nemoAttIndices,1)'/1000;
nemoAtt = phoneData(nemoAttIndices,3:5)';
nemoAtt(3,:) = nemoAtt(3,:)+mean(state(3,:))-mean(nemoAtt(3,:));

%% rotate from phone to vicon coords
% R = 1/2*[sqrt(2)    -sqrt(2)    0;
%      -sqrt(2)   -sqrt(2)    0;
%      0          0           -2];
% % R = R*diag([1 -1 -1]);
% state = blkdiag(R',R')*state;

%% rotate from vicon to phone coords
R1 = diag([1 -1 -1]);
R2 = 1/2*[sqrt(2)    -sqrt(2)    0;
     -sqrt(2)   -sqrt(2)    0;
     0          0           -2];
% R = R*diag([1 -1 -1]);
stateVicon(1:6,:) = blkdiag(R2,R2)*stateVicon(1:6,:);
stateVicon(7:12,:) = blkdiag(R2*R1, R2*R1)*stateVicon(7:12,:);

%% 
stateViconInterp = zeros(size(stateVicon,1), length(stateTime));
for i=1:size(stateViconInterp,2)
    if stateTime(i) < timeVicon(1) || stateTime(i) >= timeVicon(end)
        stateViconInterp(:,i) = stateVicon(:,1);
    else
        tL = find(timeVicon < stateTime(i),1,'last');
        tR = tL+1;
        alpha = 1-(stateTime(i)-timeVicon(tL))/(timeVicon(tR)-timeVicon(tL));
        stateViconInterp(:,i) = alpha*stateVicon(:,tL)+(1-alpha)*stateVicon(:,tR);
    end
end

%%
figure(1);
stateLabels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]' ...
              'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};
for i=1:12
    subplot(4,3,i)
    plot(timeVicon(timeVicon<=stateTime(end)), stateVicon(i,timeVicon<=stateTime(end)), stateTime, state(i,:));
    ax = axis;
    
%     line([ax(1) ax(2)],[max(state(i,:)) max(state(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
%     line([ax(1) ax(2)],[max(stateVicon(i,:)) max(stateVicon(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
    xlabel('Time [s]')
    ylabel(stateLabels(i));
end
legend('Vicon','Phone');

% figure(2);
% for i=4:6
%     subplot(3,1,i-3);
%     plot(timeVicon, stateVicon(i,:), stateTime, state(i,:));
%     xlabel('Time [s]');
%     ylabel(stateLabels(i));
% end
% 
% figure(3);
% biasLabels = {'roll bias [rad/s]' 'pitch bias [rad/s]' 'yaw bias [rad/s]'};
% for i=1:3
%     subplot(3,1,i);
%     plot(gyroBiasTime, gyroBias(i,:));
%     xlabel('Time [s]');
%     ylabel(biasLabels(i));
% end
% 
% % figure(4);
% % innovationLabels = {'x' 'y' 'z'};
% % for i=1:3
% %     subplot(2,3,i);
% %     plot(innovationTime, innovation(i,:));
% %     ax = axis;
% %     axis([innovationTime(1) innovationTime(end) ax(3) ax(4)]);
% %     line([ax(1) ax(2)],[0 0],'Color',[0 0 0],'LineStyle','--');
% %     xlabel('Time [s]');
% %     ylabel(innovationLabels(i));
% %     
% %     subplot(2,3,i+3);
% %     plot(innovationTime, innovation(i+3,:));
% %     xlabel('Time [s]');
% %     ylabel(stateLabels(i));
% % end
% 
% % figure(5)
% % uBLabels = {'uB x' 'uB y' 'uB z'};
% % for i=1:3
% %     subplot(3,1,i)
% %     plot(uBTime, uB(i,:));
% %     xlabel('Time [s]');
% %     ylabel(uBLabels(i));
% % end
% % 
% % figure(6)
% % vBLabels = {'vB x' 'vB y' 'vB z'};
% % for i=1:3
% %     subplot(3,1,i)
% %     plot(vBTime, vB(i,:));
% %     xlabel('Time [s]');
% %     ylabel(vBLabels(i));
% % end
% 
% % figure(7)
% % errLabels = {'roll err [rad]', 'pitch err [rad]', 'yaw err [rad]'};
% % stateErr = state(1:6,:) - stateViconInterp(1:6,:);
% % for i=1:3
% %     subplot(3,1,i)
% %     stateErr(i,:) = stateErr(i,:)-mean(stateErr(i,1:50));
% %     plot(stateTime,stateErr(i,:));
% %     xlabel('Time [s]');
% %     ylabel(errLabels(i));
% % end
% 
% figure(5); clf
% stateLabels = {'roll [rad]' 'pitch [rad]' 'yaw [rad]' 'roll rate [rad/s]' 'pitch rate [rad/s]' 'yaw rate [rad/s]'};
% for i=1:3
%     subplot(3,1,i)
%     plot(timeVicon(timeVicon<=stateTime(end)), stateVicon(i,timeVicon<=stateTime(end))); hold all
%     plot(stateTime, state(i,:));
%     plot(nemoAttTime, nemoAtt(i,:));
%     hold off
%     ax = axis;
%     if i < 3
%         axis([ax(1) ax(2) -0.3 0.3]);
%     else
%         axis([ax(1) ax(2) -0.4 0.4]);
%     end
%     ax = axis;
%     for j=1:length(yawResetTime)
%         line([yawResetTime(j) yawResetTime(j)],[ax(3) ax(4)],'Color','r','LineStyle',':');
%     end    
% %     line([ax(1) ax(2)],[max(state(i,:)) max(state(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
% %     line([ax(1) ax(2)],[max(stateVicon(i,:)) max(stateVicon(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
%     xlabel('Time [s]')
%     ylabel(stateLabels(i));
% end