clear

pcFile = 'runData/pcData.txt';
pcData = importdata(pcFile,'\t');
pcData = pcData(1:end-1,:);

syncIndex = 1;
% syncIndex = find(pcData(:,2) == -500,1,'last');

desStateIndices = syncIndex-1+find(pcData(syncIndex:end,2) == 2);
desStateTime = pcData(desStateIndices,1)'/1000;
desState = pcData(desStateIndices,3:end)';

viconStateIndices = syncIndex-1+find(pcData(syncIndex:end,2) == 1);
viconStateTime = pcData(viconStateIndices,1)'/1000;
viconState = pcData(viconStateIndices,3:end)';

%% filtered state derivatives
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
% figure(1);
% set(gcf,'Name','State Tracking')
% for st=1:12
%     subplot(4,3,st)
% %     plot(desStateTime,desState(st,:)); hold all
%     plot(viconStateTime,viconState(st,:)); hold all
%     hold off
% 	ax = axis; axis([viconStateTime(1) viconStateTime(end) ax(3) ax(4)]);
% end

%%
filename = 'runData/pcData_fullState.txt';
fid = fopen(filename,'w');
for i=1:length(viconStateTime)
	line = sprintf('%i\t1',viconStateTime(i)*1000);
	for st=1:12
		line = sprintf('%s\t%f',line, viconState(st,i));
	end
	fprintf(fid,'%s\n',line);
end
fclose(fid);