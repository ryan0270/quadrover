clear

%% Get vicon data
pcFile = 'runData/pcData.txt';
pcData = importdata(pcFile,'\t');
pcData = pcData(1:end-1,:);

syncIndex = 1;
% syncIndex = find(pcData(:,2) == -500,1,'last');

desViconStateIndices = syncIndex-1+find(pcData(syncIndex:end,2) == 2);
desViconStateTime = pcData(desViconStateIndices,1)'/1000;
desViconState = pcData(desViconStateIndices,3:end)';

viconStateIndices = syncIndex-1+find(pcData(syncIndex:end,2) == 1);
viconStateTime = pcData(viconStateIndices,1)'/1000;
viconState = pcData(viconStateIndices,3:end)';

%% Get phone data
phoneFile = 'runData/phoneLog.txt';
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');

angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1002);
angleStateTime = phoneData(angleStateIndices,1)'/1000;
angleState = phoneData(angleStateIndices,3:8)';

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1012);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

tranStateInterp = interp1(tranStateTime,tranState',angleStateTime)';
stateTime = angleStateTime;
state = [angleState; tranStateInterp];
state_dt = mean(diff(stateTime));

gyroBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == -1003);
gyroBiasTime = phoneData(gyroBiasIndices,1)'/1000;
gyroBias = phoneData(gyroBiasIndices,3:5)';
gyroBias_dt = mean(diff(gyroBiasTime));

pressureHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 1234);
pressureHeightTime = phoneData(pressureHeightIndices,1)'/1000;
pressureHeight = phoneData(pressureHeightIndices,3:4)';

%% rotate from vicon to phone coords
R1 = diag([1 -1 -1]);
R2 = 1/2*[sqrt(2)    -sqrt(2)    0;
     -sqrt(2)   -sqrt(2)    0;
     0          0           -2];
% R = R*diag([1 -1 -1]);
viconState(1:6,:) = blkdiag(R2,R2)*viconState(1:6,:);
viconState(7:12,:) = blkdiag(R2*R1, R2*R1)*viconState(7:12,:);

%%
vicon_dt = mean(diff(viconStateTime));
for st=4:6
	viconState(st,:) = [0 1/vicon_dt*diff(viconState(st-3,:))];
end
for st=10:12
	viconState(st,:) = [0 1/vicon_dt*diff(viconState(st-3,:))];
end

%%
figure(1);
stateLabels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]' ...
              'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};
for i=1:12
    subplot(4,3,i)
	mask = viconStateTime <= stateTime(end);
    plot(viconStateTime(mask), viconState(i,mask)); hold all
	plot(stateTime, state(i,:)); hold all
	hold off
	ax = axis; axis([stateTime(1) stateTime(end) ax(3) ax(4)]);
    
%     line([ax(1) ax(2)],[max(state(i,:)) max(state(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
%     line([ax(1) ax(2)],[max(viconState(i,:)) max(viconState(i,:))],'Color',0.5*[1 1 1],'LineStyle','--');
    xlabel('Time [s]')
    ylabel(stateLabels(i));
end
% legend('Vicon','Phone');

%%
if ~isempty(gyroBias)
	figure(3);
	biasLabels = {'roll bias [rad/s]' 'pitch bias [rad/s]' 'yaw bias [rad/s]'};
	for i=1:3
		subplot(3,1,i);
		plot(gyroBiasTime, gyroBias(i,:));
		xlabel('Time [s]');
		ylabel(biasLabels(i));
	end
end

%%
if ~isempty(pressureHeight)
	figure(1234);
	mask = viconStateTime <= pressureHeightTime(end);
	plot(viconStateTime(mask), viconState(9,mask)); hold all
	plot(pressureHeightTime, pressureHeight(1,:));
	plot(pressureHeightTime, pressureHeight(2,:));
	hold off
	xlabel('Time [s]');
	ylabel('Height [m]');
	legend('Vicon','Press','Press Comp');
end