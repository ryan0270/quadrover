disp('start chadding')

%%
viconFile = '../pcData_fullState.txt';
viconData = importdata(viconFile,'\t',0);

viconStateIndices = find(viconData(:,2) == 1);
viconStateTime = viconData(viconStateIndices,1)'/1000;
viconState = viconData(viconStateIndices,3:14)';

orbFile = '../orbResults.txt';
orbData = importdata(orbFile,'\t',0);

orbHeightIndices = find(orbData(:,2) == 99);
orbHeightTime = orbData(orbHeightIndices,1)'/1000;
orbHeight = orbData(orbHeightIndices,3)';

orbVelIndices = find(orbData(:,2) == 98);
orbVelTime = orbData(orbVelIndices,1)'/1000;
orbVel = orbData(orbVelIndices,3:5)';

mapFile = '../mapResults.txt';
mapData = importdata(mapFile,'\t',0);

mapHeightIndices = find(mapData(:,2) == 99);
mapHeightTime = mapData(mapHeightIndices,1)'/1000;
mapHeight = mapData(mapHeightIndices,3)';

mapVelIndices = find(mapData(:,2) == 98);
mapVelTime = mapData(mapVelIndices,1)'/1000;
mapVel = mapData(mapVelIndices,3:5)';

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

%%
viconStateInterporb = interp1(viconStateTime,viconState',orbVelTime)';
viconStateInterpMap = interp1(viconStateTime,viconState',mapVelTime)';

%% mean shift
orbVel = orbVel+diag(mean(viconStateInterporb(10:12,:)-orbVel,2))*ones(size(orbVel));
orbHeight = orbHeight+diag(mean(-viconStateInterporb(9,:)-orbHeight,2))*ones(size(orbHeight));

mapVel = mapVel+diag(mean(viconStateInterpMap(10:12,:)-mapVel,2))*ones(size(mapVel));
mapHeight = mapHeight+diag(mean(-viconStateInterpMap(9,:)-mapHeight,2))*ones(size(mapHeight));


%%
rmsOrbVel = rms(viconStateInterporb(10:12,:)-orbVel,2);
rmsMapVel = rms(viconStateInterpMap(10:12,:)-mapVel,2);
rmsOrbHeight = rms(-viconStateInterporb(9,:)-orbHeight);
rmsMapHeight = rms(-viconStateInterpMap(9,:)-mapHeight);

disp('vel:')
disp([rmsOrbVel rmsMapVel]);

disp('height:')
disp([rmsOrbHeight rmsMapHeight]);


%%
figure(1); clf
% set(gcf,'Units','Inches');
% curPos = get(gcf,'Position'); figSize = [5 5];
% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
stateLabels = {'x vel [m/s]', 'y vel [m/s]', 'z vel [m/s]'};
for st=1:3
	subplot(3,1,st)
	plot(viconStateTime, viconState(st+9,:)); hold all
	plot(orbVelTime, orbVel(st,:),'.'); hold all
	plot(mapVelTime, mapVel(st,:),'.'); hold all
	ax = axis;
	axis([orbVelTime(1) orbVelTime(end) ax(3) ax(4)]);
	xlabel('Time [s]');
	ylabel(stateLabels{st})
end
legend('Vicon','ORB','MAP');

% figure(2); clf
% plot(viconStateTime, -viconState(9,:)); hold all
% plot(orbHeightTime, orbHeight, '.'); hold all
% plot(mapHeightTime, mapHeight, '.'); hold all
% xlabel('Time [s]');
% ylabel('Height [m]');
% legend('Vicon','ORB','MAP');


%%
disp('chad acomplished');