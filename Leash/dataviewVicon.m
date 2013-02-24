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

%%
figure(1);
set(gcf,'Name','State Tracking')
for st=1:12
    subplot(4,3,st)
%     plot(desStateTime,desState(st,:)); hold all
    plot(viconStateTime,viconState(st,:)); hold all
    hold off
	ax = axis; axis([viconStateTime(1) viconStateTime(end) ax(3) ax(4)]);
end
