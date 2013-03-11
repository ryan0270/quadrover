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
angleState = phoneData(angleStateIndices,3:8)';

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_TRANS_STATE);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

if ~isempty(tranState)
	tranStateInterp = interp1(tranStateTime,tranState',angleStateTime)';
	stateTime = angleStateTime;
	state = [angleState; tranStateInterp];
	state_dt = mean(diff(stateTime));
end

gyroBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_GYRO_BIAS);
gyroBiasTime = phoneData(gyroBiasIndices,1)'/1000;
gyroBias = phoneData(gyroBiasIndices,3:5)';
gyroBias_dt = mean(diff(gyroBiasTime));

% pressureHeightIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == 1234);
% pressureHeightTime = phoneData(pressureHeightIndices,1)'/1000;
% pressureHeight = phoneData(pressureHeightIndices,3:4)';

opticFlowVelLSIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OPTIC_FLOW_LS);
opticFlowVelLSTime = phoneData(opticFlowVelLSIndices,1)'/1000;
opticFlowVelLS = phoneData(opticFlowVelLSIndices,3:5)';

opticFlowVelIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OPTIC_FLOW);
opticFlowVelTime = phoneData(opticFlowVelIndices,1)'/1000;
opticFlowVel = phoneData(opticFlowVelIndices,3:5)';

viconReceiveIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_RECEIVE_VICON);
viconReceiveTime = phoneData(viconReceiveIndices,1)'/1000;
viconReceive = phoneData(viconReceiveIndices,3:14)';

attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_ATT_BIAS);
attBiasTime = phoneData(attBiasIndices,1)'/1000;
attBias = phoneData(attBiasIndices,3:5)';

forceScalingIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_FORCE_GAIN);
forceScalingTime = phoneData(forceScalingIndices,1)'/1000;
forceScaling = phoneData(forceScalingIndices,3)';

motorIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_MOTOR_CMDS);
motorTime = phoneData(motorIndices,1)'/1000;
motorCmd = phoneData(motorIndices,3:6)';

%% rotate from vicon to phone coords
RotViconToQuad = createRotMat(1, pi);
RotQuadToPhone = createRotMat(3,-pi/4)*...
			  	 createRotMat(1,pi);
RotCamToPhone = createRotMat(3,-pi/2)*...
				createRotMat(1,pi);
RotPhoneToCam = RotCamToPhone';
RotViconToPhone = RotQuadToPhone*RotViconToQuad;
R1 = diag([1 -1 -1]);
R2 = 1/2*[sqrt(2)    -sqrt(2)    0;
     -sqrt(2)   -sqrt(2)    0;
     0          0           -2];
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
end

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
if exist('opticFlowVel','var') && ~isempty(opticFlowVel)
% 	opticFlowVelLS = opticFlowVelLS([2 1 3],:);
% 	opticFlowVel = opticFlowVel([2 1 3],:);
	opticFlowVelLabels = {'xDot [m/s]','yDot [m/s]','zDot [m/s]'};
	figure(12345); clf; set(gcf,'Name','Bayesian Optical Flow');
	for i=1:3
		subplot(3,1,i);
		mask  = (viconStateTime >= opticFlowVelTime(1)) .* ...
				(viconStateTime <= opticFlowVelTime(end));
		mask = find(mask);
		plot(viconStateTime(mask), viconState(i+9,mask)); hold all
		plot(opticFlowVelTime, opticFlowVel(i,:),'.'); hold all
		plot(opticFlowVelLSTime, opticFlowVelLS(i,:),'x'); hold all
		hold off
		xlabel('Time [s]');
		ylabel(opticFlowVelLabels{i});
	end
	legend('Vicon','BOF','OF LS');
	
% 	figure(12346); clf;
% 	for i=1:3
% 		subplot(3,1,i);
% 		mask  = (stateTime >= opticFlowVelTime(1)) .* ...
% 				(stateTime <= opticFlowVelTime(end));
% 		mask = find(mask);
% 		plot(stateTime(mask), state(i+9,mask)); hold all
% 		plot(opticFlowVelTime, opticFlowVel(i,:),'.'); hold all
% 		hold off
% 		xlabel('Time [s]');
% 		ylabel(opticFlowVelLabels{i});
% 	end
% 	legend('KF','BOF');
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
	legend('Vicon Meas','Phoen Received');
end

%%
kfBlindTime = tranStateTime;
kfBlind = zeros(6,length(kfBlindTime));
kfBlind(:,1) = viconReceive(7:12,1);
% measVar = diag([0.02 0.02 0.02 0.01 0.01 0.01].^2);
% dynVar = diag([0.01 0.01 0.01 0.1 0.1 0.2].^2);
measVar = diag([0.0002 0.0002 0.0002 0.01 0.01 0.01]);
dynVar = diag([1e-4 1e-4 1e-4 0.01 0.01 0.02]);
mass = 1.1;
mask = 1+find(diff(attBiasTime)>0);
attBiasInterp = interp1(attBiasTime(mask),attBias(:,mask)',kfBlindTime,[],'extrap')';
mask = 1+find(diff(forceScalingTime)>0);
forceScalingInterp = interp1(forceScalingTime(mask), forceScaling(mask), kfBlindTime,[],'extrap');
motorCmdInterp = interp1(motorTime, motorCmd', kfBlindTime,[],'extrap')';
attHist = interp1(angleStateTime, angleState(1:3,:)', kfBlindTime,[],'extrap')';
S = 1e-4*eye(6);
lastMeasIndex = -1;
for i=1:length(kfBlindTime)-1
	dt = kfBlindTime(i+1)-kfBlindTime(i);
	att = attHist(:,i);
	attBias1 = attBiasInterp(:,i);
	forceGain = forceScalingInterp(i);
	s1 = sin(att(3)); c1 = cos(att(3));
	s2 = sin(att(2)-attBias1(2)); c2 = cos(att(2)-attBias1(2));
	s3 = sin(att(1)-attBias1(1)); c3 = cos(att(1)-attBias1(1));
	r = [s1*s3+c1*c3*s2;
		 c3*s1*s2-c1*s3;
		 c2*c3];
	 
	 thrust = forceGain*sum(motorCmdInterp(:,i));
	 if norm(thrust) < 1e-3
		 accel = zeros(3,1);
	 else
		 accel = thrust/mass*r;
		 accel(3) = accel(3)-9.81;
	 end
	 
	 % time update
	 A = [eye(3) dt*eye(3);
		  zeros(3) eye(3)];
	 B = [zeros(3); dt*eye(3)];
	 kfState = A*kfBlind(:,i)+B*accel;
	 S = A*S*A'+dynVar;
	 
	 % see if there's a new position measurement
	 [minTime, minIndex] = min(abs(viconReceiveTime-kfBlindTime(i)));
	 if minTime < kfBlindTime(i) && minIndex > lastMeasIndex
		 meas = viconReceive(7:9,minIndex);
		 lastMeasIndex = minIndex;
		 
		 C = [eye(3) zeros(3)];
		 cov = measVar(1:3,1:3);
		 m1_T = (S*C')';
		 m2_T = (C*S*C'+cov)';		 
		 K = (m2_T\m1_T)';
		 
		 err = meas-C*kfState;
		 kfState = kfState+K*err;
		 S = (eye(6)-K*C)*S;
	 end
	 
	 kfBlind(:,i+1) = kfState;
end

%%
figure(20508); clf; set(gcf,'Name','Blind KF');
for i=4:6
	subplot(3,1,i-3);
% 	mask = viconStateTime <= stateTime(end);
	mask = viconStateTime < 120;
	plot(viconStateTime(mask),viconState(i+6,mask)); hold all
	mask = stateTime < 120;
	plot(stateTime(mask),state(i+6,mask)); hold all
	mask = kfBlindTime<120;
	plot(kfBlindTime(mask), kfBlind(i,mask)); hold all
	hold off
	xlabel('Time [s]');
	ylabel(stateLabels{i+6});
end
% legend('Vicon','KF','KF Blind');