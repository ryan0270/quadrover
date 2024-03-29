clear;
disp('start chadding')

%%
% dataDir = '../dataSets/Sep8';
% dataDir = '../dataSets/Sep12';
% dataDir = '../dataSets/Sep19';
% dataDir = '../dataSets/Sep23';
% dataDir = '../dataSets/Oct3_2';
% dataDir = '../dataSets/Nov1';
% dataDir = '../dataSets/Nov2';
% dataDir = '../dataSets/Nov13_3';
dataDir = '../dataSets/Nov28';
viconFile = [dataDir '/pcData.txt'];
viconData = importdata(viconFile,'\t',0);

run([dataDir '/log_ids'])

viconStateIndices = find(viconData(:,2) == 1);
viconStateTime = viconData(viconStateIndices,1)'/1000;
viconState = viconData(viconStateIndices,3:14)';

% mask = find(abs(viconState(3,:)) > 2);
% viconStateTime(mask) = [];
% viconState(:,mask) = [];

%%
phoneFile = [dataDir '/obsvLog.txt'];
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');

% angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_ATT);
% angleStateTime = phoneData(angleStateIndices,1)'/1000;
% % angleState = phoneData(angleStateIndices,4:9)';
% quatState = phoneData(angleStateIndices,4:10)';
% temp = quat2angle(quatState(1:4,:));
% angleState = [temp([3 2 1],:); quatState(5:7,:)];

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_TRANS_STATE);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';
% accelBiasTime = tranStateTime;
% accelBias = phoneData(tranStateIndices,9:11)';

% attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_ATT_BIAS);
% attBiasTime = phoneData(attBiasIndices,1)'/1000;
% attBias = phoneData(attBiasIndices,3:5)';

mapHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MAP_HEIGHT);
mapHeightTime = phoneData(mapHeightIndices,1)'/1000;
mapHeight = phoneData(mapHeightIndices,3)';

mapVelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MAP_VEL);
mapVelTime = phoneData(mapVelIndices,1)'/1000;
mapVel = phoneData(mapVelIndices,3:5)';

% velCmdIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_VEL_CMD);
% velCmdTime = phoneData(velCmdIndices,1)'/1000;
% velCmd = phoneData(velCmdIndices,3:5)';

% accelCmdIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_ACCEL_CMD);
% accelCmdTime = phoneData(accelCmdIndices,1)'/1000;
% accelCmd = phoneData(accelCmdIndices,3:5)';

% targetAcqIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_ACQUIRED);
% targetAcqTime = phoneData(targetAcqIndices,1)'/1000;
% targetAcq = phoneData(targetAcqIndices,3:8)';

% targetLostIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_LOST);
% targetLostTime = phoneData(targetLostIndices,1)'/1000;
% targetLost = phoneData(targetLostIndices,3:8)';

% targetLocIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_FIND_CENTERS);
% targetLocTime = phoneData(targetLocIndices,1)'/1000;
% targetLoc = phoneData(targetLocIndices,3:8)';

% imageOffsetIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_IMAGE_OFFSET);
% imageOffsetTime = phoneData(imageOffsetIndices,1)'/1000;
% imageOffset = phoneData(imageOffsetIndices,4:5)';

camPosIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TARGET_ESTIMATED_POS);
camPosTime = phoneData(camPosIndices,1)'/1000;
camPos = phoneData(camPosIndices,3:5)';

% attInnovationIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_ANG_INNOVATION);
% attInnovationTime = phoneData(attInnovationIndices,1)'/1000;
% attInnovation = phoneData(attInnovationIndices,3:5)';

% torqueCmdIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_TORQUE_CMD);
% torqueCmdTime = phoneData(torqueCmdIndices,1)'/1000;
% torqueCmd = phoneData(torqueCmdIndices,3:6)';

% angleRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_REF_ATTITUDE_SYSTEM_STATE);
% angleRefTime = phoneData(angleRefIndices,1)'/1000;
% angleRef = phoneData(angleRefIndices,3:8)';
% 
% angleStateRefIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_DES_ATT);
% angleStateRefTime = phoneData(angleStateRefIndices,1)'/1000;
% angleStateRef = phoneData(angleStateRefIndices,3:8)';

trackingStatsIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBJECT_TRACKING_STATS);
trackingStatsTime = phoneData(trackingStatsIndices,1)'/1000;
trackingStats = phoneData(trackingStatsIndices,3:6)';

% useViconYawIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_USE_VICON_YAW);
% useViconYawTime = phoneData(useViconYawIndices,1)'/1000;
% 
% useViconXYIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_USE_VICON_XY);
% useViconXYTime = phoneData(useViconXYIndices,1)'/1000;

sonarHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_SONAR_HEIGHT);
sonarHeightTime = phoneData(sonarHeightIndices,1)'/1000;
sonarHeight = phoneData(sonarHeightIndices,4)';


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
angleStateLabels = {'Roll [rad]' 'Pitch [rad]' 'Yaw [rad]' 'Roll Rate [rad/s]' 'Pitch Rate [rad/s]' 'Yaw Rate [rad/s]'};
tranStateLabels = { 'x [m]' 'y [m]' 'z [m]' 'x vel [m/s]' 'y vel [m/s]' 'z vel [m/s]'};

%%
if exist('angleState','var') && ~isempty(angleState)
  	figure(1); clf;
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	
	mask = find( (viconStateTime > angleStateTime(1)) .* (viconStateTime <= angleStateTime(end) ) );
	timeShift = 0.02;
	shift = zeros(6,1);
	for i=1:6
		subplot(2,3,i);		
		plot(viconStateTime(mask), viconState(i,mask)); hold all
		plot(angleStateTime, angleState(i,:)); hold all
		hold off
		
		xlabel('Time [s]')
		ylabel(angleStateLabels(i));	
	end

% 	midPoint = round(length(angleStateTime)/2);
% 	shift = mean(angleState(:,midPoint:end),2);
% 	mask2 = mask( viconStateTime(mask) >= angleStateTime(midPoint) );	
% 	for i=1:6
% 		subplot(2,3,i);
% 		plot(viconStateTime(mask)-timeShift, viconState(i,mask)-mean(viconState(i,mask2))); hold all
% 		plot(angleStateTime, angleState(i,:)-shift(i)); hold all
% 		hold off
% % 		if i <= 2
% % 			ax = axis; axis([angleStateTime(1) angleStateTime(end) -0.6 0.6]);
% % 		else
% 			ax = axis; axis([angleStateTime(1) angleStateTime(end) ax(3) ax(4)]);
% % 		end
% 		grid on
% 
% 		xlabel('Time [s]')
% 		ylabel(angleStateLabels(i));
% 	end
	
	viconStateAngleInterp = interp1(viconStateTime, viconState', angleStateTime+timeShift,[],'extrap')';
	start = max([find(angleStateTime > angleStateTime(1)+15,1,'first');
				 0*find(angleStateTime(1,:) > 0.05,1,'first');
				 0*find(angleStateTime > mapVelTime(1),1,'first')
				 1]);
start = 1;			 
	stop = find(angleStateTime < angleStateTime(end)-5,1,'last');
	err = viconStateAngleInterp(1:3,start:stop)-angleState(1:3,start:stop);
	err = err-diag(mean(err,2))*ones(size(err));
	rmsErr = rms(err')';
	fprintf('Angle state rms err:\t');
	for i=1:3
		fprintf('%1.3f\t',rmsErr(i));
	end
	fprintf('\n')
	fprintf('            max err:\t')
	for i=1:3
		fprintf('%1.3f\t',max(abs(err(i,:))));
	end
	fprintf('\n');
end

%%
if exist('tranState','var') && ~isempty(tranState)
  	figure(2); clf;
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	mask = find( (viconStateTime > tranStateTime(1)) .* (viconStateTime <= tranStateTime(end) ) );
	timeShift = 0.0;
	for i=1:6
		subplot(2,3,i);		
		plot(viconStateTime(mask)+timeShift, viconState(i+6,mask)); hold all
		plot(tranStateTime, tranState(i,:)); hold all
		if i<=2
			plot(camPosTime,camPos(i,:),'.'); hold all
		end
		if i == 3 && ~isempty(mapHeight)
			plot(mapHeightTime, mapHeight,'.'); hold all
		elseif i>3 && ~isempty(mapVel)
			plot(mapVelTime, mapVel(i-3,:), '.'); hold all
		end
		if i == 3 && exist('sonarHeight','var') && ~isempty(sonarHeight)
			plot(sonarHeightTime, sonarHeight,'.'); hold all
		end
		hold off
% 		axis([tranStateTime(1) tranStateTime(end) -0.4 0.4]);
% 		ax = axis;
% 		for j=1:length(targetAcqTime)
% 			line([targetAcqTime(j) targetAcqTime(j)],[ax(3) ax(4)],'Color','k','LineStyle','-');
% 		end
% 		for j=1:length(targetLostTime)
% 			line([targetLostTime(j) targetLostTime(j)],[ax(3) ax(4)],'Color','k','LineStyle','--');
% 		end

		ax = axis; axis([tranStateTime(1) tranStateTime(end) ax(3) ax(4)]);

		xlabel('Time [s]')
		ylabel(tranStateLabels(i));
	end	
	
	viconStateTranInterp = interp1(viconStateTime, viconState', tranStateTime-timeShift,[],'extrap')';
	start = max([find(tranStateTime > 22,1,'first');
				 find(abs(tranState(3,:)) > 0.5,1,'first');
				 find(tranStateTime > mapVelTime(1),1,'first')]);
	rmsErr = rms(viconStateTranInterp(7:12,start:end)'-tranState(:,start:end)')';
	fprintf('Tran state rms err:\t');
	for i=1:6
		fprintf('%1.3f\t',rmsErr(i));
	end
	fprintf('\n');
	
	if ~isempty(mapVel)
		viconStateMAPInterp = interp1(viconStateTime, viconState', mapVelTime-timeShift,[],'extrap')';
		start = find(viconStateMAPInterp(9,:) > 0.5,1,'first');
		rmsErrHeight = rms(viconStateMAPInterp(9,start:end)-mapHeight(start:end));
		rmsErrVel = rms(viconStateMAPInterp(10:12,start:end)' - mapVel(:,start:end)')';
		fprintf('MAP vel rms err:\t');
		fprintf('---\t---\t%1.3f\t',rmsErrHeight);
		for i=1:3
			fprintf('%1.3f\t',rmsErrVel(i));
		end
		fprintf('\n');
	end
end

%%
if exist('accelBias','var') && ~isempty(accelBias)
  	figure(3); clf;
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	for i=1:3
		plot(accelBiasTime, accelBias(i,:)); hold all
	end
	hold off
	xlabel('Time [s]')
	ylabel('Accel bias [m/s^2]');
	legend('x','y','z','location','best');
end

%%
if exist('accelCmd','var') && ~isempty(accelCmd)
	figure(5); clf
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	accelCmdTemp = accelCmd;
	accelCmdTemp(3,:) = accelCmdTemp(3,:)-9.81;
	plot(accelCmdTime, accelCmdTemp');hold all
	ax = axis;
	plot([ax(1) ax(2)],[0 0],'k--'); hold all
	hold off
	xlabel('Time [s]');
	ylabel('Accel cmd [m/s^2]')
	legend('x','y','z');
end

%%
if exist('velCmd','var') && ~isempty(velCmd)
	figure(685); clf
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	plot(velCmdTime, velCmd');hold all
	ax = axis;
	plot([ax(1) ax(2)],[0 0],'k--'); hold all
	hold off
	xlabel('Time [s]');
	ylabel('Vel cmd [m/s^2]')
	legend('x','y','z');
end

%%
% chadTime = targetLocTime;
% chadPos = zeros(2,length(chadTime));
% 
% f = 524.005870/2;
% center = [317.122191; 248.851692]/2;
% 
% mask = [1 1+find(diff(tranStateTime) > 0)];
% tranStateTime = tranStateTime(mask);
% tranState = tranState(:,mask);
% 
% mask = [1 1+find(diff(angleStateTime) > 0)];
% angleStateTime = angleStateTime(mask);
% angleState = angleState(:,mask);
% 
% p = nan(3,1);
% for i=1:length(chadTime)
% 	att = interp1(angleStateTime, angleState(1:3,:)', chadTime(i))';
% 	R = createRotMat(att(3), att(2), att(1));
% 	p(1) = targetLoc(1,i)-center(1);
% 	p(2) = targetLoc(2,i)-center(2);
% 	p(3) = f;
% 	p = -R*RotCamToPhone*p;
% 	
% 	z = interp1(tranStateTime, tranState(3,:), chadTime(i));
% 	x = p(1)/f*z;
% 	y = p(2)/f*z;
% 	chadPos(1,i) = x;
% 	chadPos(2,i) = y;
% 	chadPos(3,i) = z;
% end
% 
% figure(2020); clf
% mask1 = find( (viconStateTime > chadTime(1)) .* (viconStateTime < chadTime(end)) );
% mask2 = find( (tranStateTime > chadTime(1))  .* (tranStateTime < chadTime(end)) );
% for i=1:3
% 	subplot(3,1,i)
% 	plot(viconStateTime(mask1), viconState(i+6,mask1)); hold all
% 	plot(tranStateTime(mask2), tranState(i,mask2)); hold all
% 	plot(chadTime, chadPos(i,:),'.'); hold all
% 	xlabel('Time [s]');
% 	ylabel(tranStateLabels{i});
% 	hold off
% end
% legend('vicon','kf','cam');

%%
if exist('imageOffset','var') && ~isempty(imageOffset)
	figure(8000);
	plot(imageOffset(1,:), imageOffset(2,:));
	xlabel('x');
	ylabel('y');
end

%%
if exist('attInnovation','var') && ~isempty(attInnovation)
	figure(8907348);
	for i=1:3
		subplot(1,3,i)
		plot(attInnovationTime, attInnovation(i,:));
		xlabel('Time');
		ylabel('Innovation');
	end
end

%%
if exist('torqueCmd','var') && ~isempty(torqueCmd)
	figure(9000);
	plot(torqueCmdTime, torqueCmd(1:3,:));
	xlabel('Time [s]');
	ylabel('Torque cmd [Nm/s]');
end

%%
if exist('angleStateRef','var') && ~isempty(angleStateRef)
% 	figure(9001); clf
% 	plot(angleStateRefTime, angleStateRef(1:3,:));
% 	xlabel('Time [s]');
% 	ylabel('Ref');
end

%%
if exist('angleRef','var') && ~isempty(angleRef)
	figure(9005);clf
	for i=1:6
		subplot(2,3,i)
		plot(angleStateRefTime, angleStateRef(i,:)); hold all
		plot(angleStateTime, angleState(i,:)); hold all
		plot(angleRefTime, angleRef(i,:)); hold all
		xlabel('Time [s]');
		ylabel(angleStateLabels{i});
	end
	legend('Cmd','Actual','Ref')
end

%%
if exist('trackingStats','var') && ~isempty(trackingStats)
	figure(13000);
	statLabels = {'Oldest [s]', 'Median [s]', '# repeats', '# new'};
	for i=1:4
		subplot(4,1,i)
		plot(trackingStatsTime, trackingStats(i,:));
		xlabel('Time [s]');
		ylabel(statLabels{i});
	end
	
	fprintf('age stats:\t');
	for i=1:4
		fprintf('%1.2f\t',mean(trackingStats(i,:)));
	end
	fprintf('\n');
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
