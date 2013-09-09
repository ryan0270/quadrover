clear;
disp('start chadding')

%%
log_ids

%%
dataDir = '../dataSets/Sep8';
viconFile = [dataDir '/pcData.txt'];
viconData = importdata(viconFile,'\t',0);

viconStateIndices = find(viconData(:,2) == 1);
viconStateTime = viconData(viconStateIndices,1)'/1000;
viconState = viconData(viconStateIndices,3:14)';

%%
phoneFile = [dataDir '/obsvLog.txt'];
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');

angleStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_ATT);
angleStateTime = phoneData(angleStateIndices,1)'/1000;
angleState = phoneData(angleStateIndices,3:8)';

tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_TRANS_STATE);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

attBiasIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_OBSV_TRANS_ATT_BIAS);
attBiasTime = phoneData(attBiasIndices,1)'/1000;
attBias = phoneData(attBiasIndices,3:5)';

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

[b, a] = butter(5,40/nyq);
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
% if exist('angleState','var') && ~isempty(angleState)
%   	figure(1); clf;
% % 	set(gcf,'Units','Inches');
% % 	curPos = get(gcf,'Position'); figSize = [6 4];
% % 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
% 	mask = find( (viconStateTime > angleStateTime(1)) .* (viconStateTime <= angleStateTime(end) ) );
% 	for i=1:6
% 		subplot(2,3,i);		
% 		plot(viconStateTime(mask), viconState(i,mask)); hold all
% 		plot(angleStateTime, angleState(i,:)); hold all
% 		hold off
% 
% 		xlabel('Time [s]')
% 		ylabel(angleStateLabels(i));	
% 	end
% % 	% legend('Vicon','Phone');
% end

%%
if exist('tranState','var') && ~isempty(tranState)
  	figure(2); clf;
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [6 4];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
	mask = find( (viconStateTime > tranStateTime(1)) .* (viconStateTime <= tranStateTime(end) ) );
	for i=1:6
		subplot(2,3,i);		
		plot(viconStateTime(mask), viconState(i+6,mask)); hold all
		plot(tranStateTime, tranState(i,:)); hold all
		hold off

		xlabel('Time [s]')
		ylabel(tranStateLabels(i));
	end
	
	viconStateTranInterp = interp1(viconStateTime, viconState', tranStateTime,[],'extrap')';
	rmsErr = rms(viconStateTranInterp(7:12,:)-tranState,2);
	fprintf('Tran state rms err:\t');
	for i=1:6
		fprintf('%1.3f\t',rmsErr(i));
	end
	fprintf('\n');
end
