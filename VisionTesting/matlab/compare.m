disp('start chadding')

%%
imgDir = '../video_Jun5_3';
viconFile = [imgDir '/pcData_fullState.txt'];
viconData = importdata(viconFile,'\t',0);

viconStateIndices = find(viconData(:,2) == 1);
viconStateTime = viconData(viconStateIndices,1)'/1000;
viconState = viconData(viconStateIndices,3:14)';

phoneFile = [imgDir '/log.txt'];
phoneData = importdata(phoneFile,'\t');
phoneData = phoneData(1:end-1,:);

syncIndex = find(phoneData(:,2) == -500,1,'last');
LOG_ID_CUR_TRANS_STATE=-1012;
tranStateIndices = syncIndex-1+find(phoneData(syncIndex:end,2) == LOG_ID_CUR_TRANS_STATE);
tranStateTime = phoneData(tranStateIndices,1)'/1000;
tranState = phoneData(tranStateIndices,3:8)';

orbFile = '../orbResults.txt';
orbData = importdata(orbFile,'\t',0);

orbHeightIndices = find(orbData(:,2) == 99);
orbHeightTime = orbData(orbHeightIndices,1)'/1000;
orbHeight = orbData(orbHeightIndices,3)';

orbVelIndices = find(orbData(:,2) == 98);
orbVelTime = orbData(orbVelIndices,1)'/1000;
orbVel = orbData(orbVelIndices,3:5)';

orbKfStateIndices = find(orbData(:,2) == 100);
orbKfStateTime = orbData(orbKfStateIndices,1)'/1000;
orbKfState = orbData(orbKfStateIndices,3:8)';

lsVelIndices = find(orbData(:,2) == 101);
lsVelTime = orbData(lsVelIndices,1)'/1000;
lsVel = orbData(lsVelIndices,3:5)';

mapFile = '../mapResults.txt';
mapData = importdata(mapFile,'\t',0);

mapHeightIndices = find(mapData(:,2) == 99);
mapHeightTime = mapData(mapHeightIndices,1)'/1000;
mapHeight = mapData(mapHeightIndices,3)';

mapVelIndices = find(mapData(:,2) == 98);
mapVelTime = mapData(mapVelIndices,1)'/1000;
mapVel = mapData(mapVelIndices,3:5)';

mapKfStateIndices = find(mapData(:,2) == 100);
mapKfStateTime = mapData(mapKfStateIndices,1)'/1000;
mapKfState = mapData(mapKfStateIndices,3:8)';

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
R1 = RotQuadToPhone*RotPhoneToCam;
R2 = RotViconToQuad*RotQuadToPhone*RotPhoneToCam;

% R = R*diag([1 -1 -1]);
viconState(1:6,:) = blkdiag(R1,R1)*viconState(1:6,:);
viconState(7:12,:) = blkdiag(R2, R2)*viconState(7:12,:);

tranState = blkdiag(RotPhoneToCam, RotPhoneToCam)*tranState;
mapKfState = blkdiag(RotPhoneToCam, RotPhoneToCam)*mapKfState;
orbKfState = blkdiag(RotPhoneToCam, RotPhoneToCam)*orbKfState;

%%
% viconStateTime = viconStateTime-0.05;
viconStateInterpOrb = interp1(viconStateTime,viconState',orbVelTime,[],'extrap')';
viconStateInterpMap = interp1(viconStateTime,viconState',mapVelTime,[],'extrap')';
viconStateInterpKF = interp1(viconStateTime,viconState',mapKfStateTime,[],'extrap')';

%%
timeL = 26;
timeR = 55;
timeMaskOrb = find((orbVelTime > timeL) .* (orbVelTime < timeR));
timeMaskOrbKF = find((orbKfStateTime > timeL) .* (orbKfStateTime < timeR));
timeMaskMap = find((mapVelTime > timeL) .* (mapVelTime < timeR));
timeMaskMapKF = find((mapKfStateTime > timeL) .* (mapKfStateTime < timeR));
timeMaskLS = find((lsVelTime > timeL) .* (lsVelTime < timeR));
rmsOrbVel = rms(viconStateInterpOrb(10:12,timeMaskOrb)-orbVel(:,timeMaskOrb),2);
rmsLSVel = rms(viconStateInterpOrb(10:12, timeMaskLS)-lsVel(:,timeMaskLS),2);
rmsOrbKFVel = rms(viconStateInterpOrb(10:12,timeMaskOrbKF)-orbKfState(4:6,timeMaskOrbKF),2);
rmsMapVel = rms(viconStateInterpMap(10:12,timeMaskMap)-mapVel(:,timeMaskMap),2);
rmsMapKFVel = rms(viconStateInterpKF(10:12,timeMaskMapKF)-mapKfState(4:6,timeMaskMapKF),2);

rmsOrbHeight = rms(-viconStateInterpOrb(9,:)-orbHeight);
rmsOrbKFHeight = rms(viconStateInterpOrb(9,:)-orbKfState(3,:));
rmsMapHeight = rms(-viconStateInterpMap(9,:)-mapHeight);
rmsMapKFHeight = rms(viconStateInterpKF(9,:)-mapKfState(3,:));

disp('    LS Vel    OKF vel   MKF vel   ORB vel   MAP vel')
disp([rmsLSVel rmsOrbKFVel rmsMapKFVel rmsOrbVel rmsMapVel]);

disp('height:')
disp([0 rmsOrbKFHeight rmsMapKFHeight rmsOrbHeight rmsMapHeight]);


%%
figure(1); clf
% set(gcf,'Units','Inches');
% curPos = get(gcf,'Position'); figSize = [5 5];
% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
stateLabels = {'x [m]', 'y [m]', 'z [m]', 'x vel [m/s]', 'y vel [m/s]', 'z vel [m/s]'};
for st=4:6
	subplot(3,1,st-3)
	plot(viconStateTime, viconState(st+6,:)); hold all
% 	plot(orbVelTime, orbVel(st,:),'.'); hold all
% 	plot(mapVelTime, mapVel(st,:),'.'); hold all
	plot(orbKfStateTime, orbKfState(st,:),'.'); hold all
	plot(mapKfStateTime, mapKfState(st,:),'.'); hold all
% 	plot(lsVelTime, lsVel(st,:),'.'); hold all
	hold off
	ax = axis;
% 	axis([orbVelTime(1) orbVelTime(end) ax(3) ax(4)]);
	axis([orbVelTime(1) orbVelTime(end) -1 1]);
	xlabel('Time [s]');
	ylabel(stateLabels{st})
end
% legend('Vicon','ORB','MAP','MAP KF', 'LS');

figure(2); clf
plot(viconStateTime, -viconState(9,:)); hold all
% plot(orbHeightTime, orbHeight, '.'); hold all
% plot(mapHeightTime, mapHeight, '.'); hold all
plot(orbKfStateTime, -orbKfState(3,:), '.'); hold all
plot(mapKfStateTime, -mapKfState(3,:), '.'); hold all
hold off
xlabel('Time [s]');
ylabel('Height [m]');
% legend('Vicon','ORB','MAP');

% figure(3); clf
% for st=4:6
% 	subplot(3,1,st-3)
% 	plot(orbKfStateTime, orbKfState(st,:) - viconStateInterpOrb(st+6,:), '.', 'Color',[0 0.5 0]); hold all
% 	plot(mapKfStateTime, mapKfState(st,:) - viconStateInterpMap(st+6,:), '.r'); hold all
% 	hold off
% 	axis([orbVelTime(1) orbVelTime(end) -0.5 0.5]);
% 	xlabel('Time [s]');
% 	ylabel(stateLabels{st});
% end


%%
disp('chad acomplished');