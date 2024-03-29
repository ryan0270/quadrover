clear

%% script defining phone log ids
log_ids; 

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

angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_ATT);
angleStateTime = phoneData(angleStateIndices,1)'/1000;
% angleState = phoneData(angleStateIndices,4:9)';
quatState = phoneData(angleStateIndices,4:10)';
temp = quat2angle(quatState(1:4,:));
angleState = [temp([3 2 1],:); quatState(5:7,:)];

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_TRANS_STATE);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';
mask = [1 1+find(diff(tranStateTime)~=0)];
tranStateTime = tranStateTime(mask);
tranState = tranState(:,mask);
if ~isempty(tranState)
	tranStateInterp = interp1(tranStateTime,tranState',angleStateTime,[],'extrap')';
	stateTime = angleStateTime;
	state = [angleState; tranStateInterp];
	state_dt = mean(diff(stateTime));
end

% pressureHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 1234);
% pressureHeightTime = phoneData(pressureHeightIndices,1)'/1000;
% pressureHeight = phoneData(pressureHeightIndices,3:4)';

opticFlowVelLSIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OPTIC_FLOW_LS);
opticFlowVelLSTime = phoneData(opticFlowVelLSIndices,1)'/1000;
opticFlowVelLS = phoneData(opticFlowVelLSIndices,3:5)';

opticFlowVelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OPTIC_FLOW);
opticFlowVelTime = phoneData(opticFlowVelIndices,1)'/1000;
opticFlowVel = phoneData(opticFlowVelIndices,3:5)';
opticFlowLag = phoneData(opticFlowVelIndices,6)';

% viconReceiveIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_RECEIVE_VICON);
% viconReceiveTime = phoneData(viconReceiveIndices,1)'/1000;
% viconReceive = phoneData(viconReceiveIndices,3:14)';

attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_ATT_BIAS);
attBiasTime = phoneData(attBiasIndices,1)'/1000;
attBias = phoneData(attBiasIndices,3:5)';

forceScalingIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_FORCE_GAIN);
forceScalingTime = phoneData(forceScalingIndices,1)'/1000;
forceScaling = phoneData(forceScalingIndices,3)';

motorIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MOTOR_CMDS);
motorTime = phoneData(motorIndices,1)'/1000;
motorCmd = phoneData(motorIndices,3:6)';

ibvsOffIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_IBVS_DISABLED);
ibvsOffTime = phoneData(ibvsOffIndices,1)'/1000;
ibvsOff = phoneData(ibvsOffIndices,3:5)';

ibvsOnIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_IBVS_ENABLED);
ibvsOnTime = phoneData(ibvsOnIndices,1)'/1000;
ibvsOn = phoneData(ibvsOnIndices,3:5)';

cameraPosIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_ESTIMATED_POS);
cameraPosTime = phoneData(cameraPosIndices,1)'/1000;
cameraPos = phoneData(cameraPosIndices,3:5)';

mapVelEstIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MAP_VEL);
mapVelEstTime = phoneData(mapVelEstIndices,1)'/1000;
mapVelEst = phoneData(mapVelEstIndices,3:5)';

mapHeightEstIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MAP_HEIGHT);
mapHeightEstTime = phoneData(mapHeightEstIndices,1)'/1000;
mapHeightEst = phoneData(mapHeightEstIndices,3)';

velCmdIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_VEL_CMD);
velCmdTime = phoneData(velCmdIndices,1)'/1000;
velCmd = phoneData(velCmdIndices,3:5)';

targetAcqIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_ACQUIRED);
targetAcqTime = phoneData(targetAcqIndices,1)'/1000;
targetAcq = phoneData(targetAcqIndices,3:8)';

targetLostIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_LOST);
targetLostTime = phoneData(targetLostIndices,1)'/1000;
targetLost = phoneData(targetLostIndices,3:8)';

targetLocIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_FIND_CENTERS);
targetLocTime = phoneData(targetLocIndices,1)'/1000;
targetLoc = phoneData(targetLocIndices,3:8)';

sonarHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_SONAR_HEIGHT);
sonarHeightTime = phoneData(sonarHeightIndices,1)'/1000;
sonarHeight = phoneData(sonarHeightIndices,4)';

useViconYawIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_USE_VICON_YAW);
useViconYawTime = phoneData(useViconYawIndices,1)'/1000;

useViconXYIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_USE_VICON_XY);
useViconXYTime = phoneData(useViconXYIndices,1)'/1000;


%% rotate from vicon to phone coords
RotViconToQuad = createRotMat(1, pi);
RotQuadToPhone = createRotMat(3,-pi/4)*...
			  	 createRotMat(1,pi);
RotCamToPhone = createRotMat(3,-pi/2)*...
				createRotMat(1,pi);
RotPhoneToCam = RotCamToPhone';
RotViconToPhone = RotQuadToPhone*RotViconToQuad;
% R1 = diag([1 -1 -1]);
% R2 = 1/2*[sqrt(2)    -sqrt(2)    0;
%      -sqrt(2)   -sqrt(2)    0;
%      0          0           -2];
R1 = RotViconToQuad;
R2 = RotQuadToPhone;
% R = R*diag([1 -1 -1]);
viconState(1:6,:) = blkdiag(R2,R2)*viconState(1:6,:);
viconState(7:12,:) = blkdiag(R2*R1, R2*R1)*viconState(7:12,:);

if exist('viconReceive','var') && ~isempty(viconReceive)
	viconReceive(1:6,:) = blkdiag(R2, R2)*viconReceive(1:6,:);
	viconReceive(7:12,:) = blkdiag(R2*R1,R2*R1)*viconReceive(7:12,:);
end

%%
vicon_dt = mean(diff(viconStateTime));
nyq = 1/2/vicon_dt;
[b, a] = butter(5,20/nyq);
for st=4:6
	viconState(st,:) = [0 1/vicon_dt*diff(viconState(st-3,:))];
	viconState(st,:) = filtfilt(b,a,viconState(st,:));
% 	badMask = abs(viconState(st,:))>100;
% 	viconState(st,badMask) = 0;
end

[b, a] = butter(5,5/nyq);
for st=10:12
	viconState(st,:) = [0 1/vicon_dt*diff(viconState(st-3,:))];
	viconState(st,:) = filtfilt(b,a,viconState(st,:));
% 	badMask = abs(viconState(st,:))>5;
% 	viconState(st,badMask) = 0;
end

%%
if exist('state','var') && ~isempty(state)
	stateLabels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]' ...
				  'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};
  	figure(1); clf;
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	mask = viconStateTime <= stateTime(end);
	offset = [0 0 0 0 0 0 0 0 0.1 0.0 0.0 0];
	for i=1:12
		subplot(4,3,i)
		
		plot(stateTime, state(i,:)+offset(i),'Color',[0 0.5 0]); hold all
		if i < 7
			plot(viconStateTime(mask), viconState(i,mask),'b'); hold all
		else
			plot(viconStateTime(mask), viconState(i,mask),'b','LineWidth',2); hold all
		end
		if i == 7 || i == 8
			plot(cameraPosTime,cameraPos(i-6,:),'r.'); hold all
		end
		if i == 9
			chad = interp1(tranStateTime,tranState',sonarHeightTime)';
			mask2 = find(abs(chad(3,:)-sonarHeight) < 0.1);
			plot(sonarHeightTime(mask2), sonarHeight(mask2)+0.1, 'm.');
		end
% 		if i > 9
% 			plot(mapVelEstTime, mapVelEst(i-9,:),'.'); hold all
% 		end
		hold off
		ax = axis; axis([stateTime(1) stateTime(end) ax(3) ax(4)]);
		ax = axis;
		for j=1:length(targetAcqTime)
			line([targetAcqTime(j) targetAcqTime(j)],[ax(3) ax(4)],'Color','k','LineStyle','-');
		end
		for j=1:length(targetLostTime)
			line([targetLostTime(j) targetLostTime(j)],[ax(3) ax(4)],'Color','k','LineStyle','--');
		end
		xlabel('Time [s]')
		ylabel(stateLabels(i));
	
% 		axis([30 50 -1 1]);
	end
	% legend('Vicon','Phone');
	
	viconInterp = interp1(viconStateTime,viconState',tranStateTime,[],'extrap')';
	err = tranState-viconInterp(7:end,:)+diag(offset(7:end))*ones(size(tranState));
	rmsErr = rms(err')';
	fprintf('tran state rms err:\t');
	for i=1:6
		fprintf('%1.3f\t',rmsErr(i));
	end
	fprintf('\n');
	
	if ~isempty(mapVelEst)
		viconStateMAPInterp = interp1(viconStateTime, viconState', mapVelEstTime,[],'extrap')';
		rmsErrHeight = rms(viconStateMAPInterp(9,:)-mapHeightEst-offset(9));
		rmsErrVel = rms(viconStateMAPInterp(10:12,:)' - mapVelEst')';
		fprintf('MAP vel rms err:\t');
		fprintf('---\t---\t%1.3f\t',rmsErrHeight);
		for i=1:3
			fprintf('%1.3f\t',rmsErrVel(i));
		end
		fprintf('\n');
	end
end

%%
if exist('pressureHeight','var') && ~isempty(pressureHeight)
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

%%
if exist('opticFlowLag','var') && ~isempty(opticFlowLag)
	figure(8767);
	plot(opticFlowVelTime, opticFlowLag);
	xlabel('Time [s]');
	ylabel('Optic Flow Lag [ms]');
end

%%
if exist('viconReceive','var') && ~isempty(viconReceive)
	figure(700);
	stateLabels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]' ...
				  'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};
	mask = find(viconStateTime <= viconReceiveTime(end));
	for i=7:9
		subplot(3,1,i-6)
		plot(viconStateTime(mask), viconState(i,mask),'o'); hold all
		plot(viconReceiveTime, viconReceive(i,:),'.'); hold all
		hold off
		xlabel('Time [s]');
		ylabel(stateLabels{i});
	end
	legend('Vicon Meas','Phone Received');
end

%%
if exist('cameraPos','var') && ~isempty(cameraPos)
% 	figure(800); set(gcf,'Name','Camera Pos');
% 	mask = find(viconStateTime <= cameraPosTime(end));
% 	for i=1:3
% 		subplot(3,1,i)
% 		plot(viconStateTime(mask), viconState(i+6,mask)); hold all
% 		plot(cameraPosTime, cameraPos(i,:),'.'); hold all
% 		hold off
% 		xlabel('Time [s]');
% 		ylabel(stateLabels{i+6})
% 	end
end

%%
if exist('useViconYawTime','var')
	figure(14000); clf
	if isempty(useViconYawTime)
		time = phoneData(syncIndex-1,1)/1000:0.1:phoneData(end,1)/1000;
		plot(time, -0.25*ones(size(time)));hold all
	else
		plot(useViconYawTime,0.75*ones(size(useViconYawTime)),'o'); hold all
	end
	
	if isempty(useViconXYTime)
		time = phoneData(syncIndex-1,1)/1000:0.1:phoneData(end,1)/1000;
		plot(time, -0.25*ones(size(time)));hold all
	else
		plot(useViconXYTime,ones(size(useViconXYTime)),'x'); hold all
	end
	hold off
	
	ax = axis;
	axis([phoneData(syncIndex-1,1)/1000 phoneData(end,1)/1000 -0.5 1.5]);
	xlabel('Time');
	ylabel('Vicon Usage');
	legend('Vicon Yaw','Vicon XY');
end


%%
disp('chad accomplished')