disp('start chadding')

%%
imgDir = '../video_Jun5_3';
% imgDir = '../video_Jun10_1';
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

% disp('    LS Vel    OKF vel   MKF vel   ORB vel   MAP vel')
% disp([rmsLSVel rmsOrbKFVel rmsMapKFVel rmsOrbVel rmsMapVel]);
% 
% disp('height:')
% disp([0 rmsOrbKFHeight rmsMapKFHeight rmsOrbHeight rmsMapHeight]);

fprintf('ORB\t%1.4f & %1.4f & %1.4f & %1.4f & \n', rmsOrbKFVel(1), rmsOrbKFVel(2), rmsOrbKFVel(3), rmsOrbKFHeight);
fprintf('MAP\t%1.4f & %1.4f & %1.4f & %1.4f & \n', rmsMapKFVel(1), rmsMapKFVel(2), rmsMapKFVel(3), rmsMapKFHeight);


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

% figure(2); clf
% plot(viconStateTime, -viconState(9,:)); hold all
% % plot(orbHeightTime, orbHeight, '.'); hold all
% % plot(mapHeightTime, mapHeight, '.'); hold all
% plot(orbKfStateTime, -orbKfState(3,:), '.'); hold all
% plot(mapKfStateTime, -mapKfState(3,:), '.'); hold all
% hold off
% xlabel('Time [s]');
% ylabel('Height [m]');
% % legend('Vicon','ORB','MAP');
% 
% % figure(3); clf
% % for st=4:6
% % 	subplot(3,1,st-3)
% % 	plot(orbKfStateTime, orbKfState(st,:) - viconStateInterpOrb(st+6,:), '.', 'Color',[0 0.5 0]); hold all
% % 	plot(mapKfStateTime, mapKfState(st,:) - viconStateInterpMap(st+6,:), '.r'); hold all
% % 	hold off
% % 	axis([orbVelTime(1) orbVelTime(end) -0.5 0.5]);
% % 	xlabel('Time [s]');
% % 	ylabel(stateLabels{st});
% % end

%%
% clear
% numPoints = [3.5 7.2 18 45 80 108 162 209 271];
% 
% rmsErrKF(3,:) = 129/10000*ones(size(numPoints));
% rmsErrKF(4,:) = 843/10000*ones(size(numPoints));
% rmsErrKF(5,:) = 927/10000*ones(size(numPoints));
% rmsErrKF(6,:) = 1107/10000*ones(size(numPoints));
% 
% rmsErrORB(3,:) = [127 126 120 152 120 121 121 121 121]/1e4;
% rmsErrORB(4,:) = [854 844 1047 753 720 724 718 715 710]/1e4;
% rmsErrORB(5,:) = [928 887 867 1188 852 884 841 832 824]/1e4;
% rmsErrORB(6,:) = [1077 1059 1005 2045 1008 1064 1011 1019 1015]/1e4;
% 
% rmsErrNEW3s(3,:) = [123 121 118 116 115 114 114 114 115]/1e4;
% rmsErrNEW3s(4,:) = [703 672 666 684 680 674 659 647 647]/1e4;
% rmsErrNEW3s(5,:) = [792 754 747 741 740 736 730 735 734]/1e4;
% rmsErrNEW3s(6,:) = [1018 999 958 925 904 884 885 897 903]/1e4;
% 
% rmsErrNEW2s(3,:) = [123 121 119 116 115 114 114 115 115]/1e4;
% rmsErrNEW2s(4,:) = [705 677 670 682 684 679 667 657 654]/1e4;
% rmsErrNEW2s(5,:) = [808 768 764 746 747 749 743 743 744]/1e4;
% rmsErrNEW2s(6,:) = [1020 1005 970 930 911 892 893 901 908]/1e4;
% 
% rmsErrNEW1s(3,:) = [123 121 120 119 118 118 118 117 118]/1e4;
% rmsErrNEW1s(4,:) = [720 696 665 675 665 669 670 666 668]/1e4;
% rmsErrNEW1s(5,:) = [831 799 788 784 774 775 778 779 786]/1e4;
% rmsErrNEW1s(6,:) = [1023 1012 990 968 950 942 941 940 944]/1e4;
% 
% calcTimeORB =   [2.0 2.2 2.5 2.9 3.4 3.8 5.7 5.7 7.2];
% calcTimeNEW3s = [1.3 1.4 1.7 2.2 2.7 3.6 5.0 6.8 9.2];
% calcTimeNEW2s = [1.3 1.4 1.7 2.2 2.7 3.6 4.9 6.6 8.8];
% calcTimeNEW1s = [1.3 1.4 1.6 2.1 2.6 3.4 4.8 6.2 8.5];
% 
% stateLabels = {'x [m]', 'y [m]', 'z [m]', 'x vel [m/s]', 'y vel [m/s]', 'z vel [m/s]'};
% for st=3:6
% 	figure(10+st); clf
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [5 5];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
% 
% 	plot(numPoints, rmsErrKF(st,:)); hold all
% 	plot(numPoints, rmsErrORB(st,:), 'Color',[0 0.5 0]); hold all
% 	plot(numPoints, rmsErrNEW3s(st,:), 'r'); hold all
% 	if st == 3
% 		axis([0 numPoints(end) 0.00 0.015]);
% 	elseif st == 4 || st == 5
% 		axis([0 numPoints(end) 0.0 0.1]);
% 	else
% 		axis([0 numPoints(end) 0.0 0.12]);
% 	end
% 	hold off
% 	xlabel('# feature points');
% 	ylabel(['rms err ' stateLabels{st}]);
% 	legend('KF only','ORB Hard C','Soft C','location','best')
% end
% 
% figure(20); clf
% set(gcf,'Units','Inches');
% curPos = get(gcf,'Position'); figSize = [5 5];
% set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
% plot(numPoints, calcTimeORB, 'Color', [0 0.5 0]); hold all
% plot(numPoints, calcTimeNEW3s, 'r'); hold all
% plot(numPoints, calcTimeNEW2s, 'm'); hold all
% plot(numPoints, calcTimeNEW1s, 'k'); hold all
% hold off
% axis([0 numPoints(end) 0 10]);
% xlabel('# feature points');
% ylabel('Calc Time [ms]');
% legend('ORB Hard C', 'Soft C 3\sigma', 'Soft C 2\sigma', 'Soft C 1\sigma','location','best');
% 
% for st=3:6
% 	figure(30+st); clf
% 	set(gcf,'Units','Inches');
% 	curPos = get(gcf,'Position'); figSize = [5 5];
% 	set(gcf,'PaperSize',figSize,'PaperPosition',[0 0 figSize],'Position',[curPos(1:2) figSize]);
% 
% 	plot(numPoints, rmsErrNEW3s(st,:), 'r'); hold all
% 	plot(numPoints, rmsErrNEW2s(st,:), 'm'); hold all
% 	plot(numPoints, rmsErrNEW1s(st,:), 'k'); hold all
% 	hold off
% 	if st == 3
% 		axis([0 numPoints(end) 0.00 0.015]);
% 	elseif st == 4 || st == 5
% 		axis([0 numPoints(end) 0.0 0.1]);
% 	else
% 		axis([0 numPoints(end) 0.0 0.12]);
% 	end
% 	hold off
% 	xlabel('# feature points');
% 	ylabel(['rms err ' stateLabels{st}]);
% 	legend('Soft C 3\sigma','Soft C 2\sigma', 'Soft C 1\sigma','location','best')
% end


%%
disp('chad acomplished');