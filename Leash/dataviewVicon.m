clear

DELIMITER = ',';
HEADERLINES = 2;

% Import the file
% newData1 = importdata('runData/data.csv', DELIMITER, HEADERLINES);
newData1 = importdata('runData/Jan10/vicon_2cm_noise_222222_cntl9.csv', DELIMITER, HEADERLINES);

% Create new variables in the base workspace from those fields.
vars = fieldnames(newData1);
for i = 1:length(vars)
    assignin('base', vars{i}, newData1.(vars{i}));
end

frame           = data(:,1)';
flightMode      = data(:,2)';
time            = data(:,3)'/1000;
commandVals     = data(:,4:7)';
targetPos       = data(:,8:19)';
state           = data(:,20:31)';
stateFiltered	= data(:,32:43)';

targetPos(1:6,:) = targetPos(1:6,:)*180/pi;
state(1:6,:) = state(1:6,:)*180/pi;
stateFiltered(1:6,:) = stateFiltered(1:6,:)*180/pi;


pathStartIndex = find(targetPos(8,:)>0,1,'first');
if isempty(pathStartIndex)
    pathStartIndex = 1;
end
time = time-time(pathStartIndex);

pathStopIndex = find(abs(targetPos(9,:))>0,1,'last');

% mask = find(flightMode == 1);

% frame = frame(mask);
% flightMode = flightMode(mask);
% time = time(mask);
% commandVals = commandVals(:,mask);
% targetPos = targetPos(:,mask);
% state = state(:,mask);
% stateFilterd = stateFiltered(:,mask);

% time = time-time(1);

%%
circleErr = zeros(1,pathStopIndex-pathStartIndex+1);
for i=1:length(circleErr)
    circleErr(i) = norm(state(7:8,pathStartIndex+i-1))-1;
end

errRMS = sqrt(mean(circleErr.^2));
fprintf('Circle err RMS: %1.2f\n',errRMS);

%%
% figure(5)
% for i=1:6
%     subplot(2,3,i);
%     hold all
%     plotfft(state(i,:),1/mean(diff(time)),0,5);
%     if i < 4
%         ax = axis; axis([0 2 ax(3) ax(4)]);
%     else
%         ax = axis; axis([0 5 ax(3) ax(4)]);
%     end
%     hold off
% end

% return

%%
figure(1);
set(gcf,'Name','State Tracking')
for st=1:12
    subplot(4,3,st)
%     hold all
    plot(time,targetPos(st,:)); hold all
    plot(time,state(st,:)); 
    hold off
end

figure(30)
set(gcf,'Name','Circle Tracking')
plot3(targetPos(7,:), targetPos(8,:), targetPos(9,:)); hold all
plot3(state(7,:), state(8,:), state(9,:))
hold off
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

%%
% figure(4);
% set(gcf,'Name','Sent Commands');
% for u=1:4
%     subplot(2,2,u)
%     plot(time, commandVals(u,:));
% end
% 
% %%
% figure(50)
% set(gcf,'Name','Pos PSD')
% for i=7:9
%     subplot(3,1,i-6);
%     plotfft(state(i,:),1/mean(diff(time)));
% end